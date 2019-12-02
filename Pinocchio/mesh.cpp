/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "mesh.h"
#include "hashutils.h"
#include "utils.h"
#include "debugging.h"
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <algorithm>
#include <unordered_map>

namespace Pinocchio {
    Mesh::Mesh(const std::string &file, float weight)
    : scale(1.), withTexture(false), blendWeight(weight) {
        int i;
        #define OUT { vertices.clear(); edges.clear(); return; }
        std::ifstream obj(file.c_str());

        if(!obj.is_open()) {
            Debugging::out() << "Error opening file " << file << std::endl;
            return;
        }

        Debugging::out() << "Reading " << file << std::endl;

        if(file.length() < 4) {
            Debugging::out() << "I don't know what kind of file it is" << std::endl;
            return;
        }

        if(std::string(file.end() - 4, file.end()) == std::string(".obj"))
            readObj(obj);
        else if(std::string(file.end() - 4, file.end()) == std::string(".ply"))
            readPly(obj);
        else if(std::string(file.end() - 4, file.end()) == std::string(".off"))
            readOff(obj);
        else if(std::string(file.end() - 4, file.end()) == std::string(".gts"))
            readGts(obj);
        else if(std::string(file.end() - 4, file.end()) == std::string(".stl"))
            readStl(obj);
        else {
            Debugging::out() << "I don't know what kind of file it is" << std::endl;
            return;
        }

        //reconstruct the rest of the information
        int verts = vertices.size();

        if(verts == 0)
            return;

        for(i = 0; i < (int)edges.size(); ++i) {            //make sure all vertex indices are valid
            if(edges[i].vertex < 0 || edges[i].vertex >= verts) {
                Debugging::out() << "Error: invalid vertex index " << edges[i].vertex << std::endl;
                OUT;
            }
        }

        fixDupFaces();

        computeTopology();

        if (integrityCheck()) {
            Debugging::out() << "Successfully read " << file << ": " << vertices.size() << " vertices, " << edges.size() << " edges" << std::endl;
        }
        else {
            Debugging::out() << "Somehow read " << file << ": " << vertices.size() << " vertices, " << edges.size() << " edges" << std::endl;
        }

        computeVertexNormals();
    }

    void Mesh::computeTopology() {
        int i;
        for(i = 0; i < (int)edges.size(); ++i) {
            edges[i].prev = (i - i % 3) + (i + 2) % 3;
        }

        std::vector<std::map<int, int> > halfEdgeMap(vertices.size());
        for(i = 0; i < (int)edges.size(); ++i) {
            int v1 = edges[i].vertex;
            int v2 = edges[edges[i].prev].vertex;

            vertices[v1].edge = edges[edges[i].prev].prev;  //assign the vertex' edge

            if(halfEdgeMap[v1].count(v2)) {
                Debugging::out() << "Error: duplicate edge detected: " << v1 << " to " << v2 << std::endl;
                OUT;
            }

            halfEdgeMap[v1][v2] = i;
            if(halfEdgeMap[v2].count(v1)) {
                int twin = halfEdgeMap[v2][v1];
                edges[twin].twin = i;
                edges[i].twin = twin;
            }
        }
    }

    void Mesh::computeVertexNormals() {
        int i;
        for(i = 0; i < (int)vertices.size(); ++i)
            vertices[i].normal = Vector3();
        for(i = 0; i < (int)edges.size(); i += 3) {
            int i1 = edges[i].vertex;
            int i2 = edges[i + 1].vertex;
            int i3 = edges[i + 2].vertex;
            Vector3 normal = ((vertices[i2].pos - vertices[i1].pos) % (vertices[i3].pos - vertices[i1].pos)).normalize();
            vertices[i1].normal += normal;
            vertices[i2].normal += normal;
            vertices[i3].normal += normal;
        }
        for(i = 0; i < (int)vertices.size(); ++i) {
            vertices[i].normal = vertices[i].normal.normalize();
        }
    }

    void Mesh::normalizeBoundingBox() {
        int i;
        std::vector<Vector3> positions;
        for(i = 0; i < (int)vertices.size(); ++i) {
            positions.push_back(vertices[i].pos);
        }
        Rect3 boundingBox = Rect3(positions.begin(), positions.end());
        double cscale = .9 / boundingBox.getSize().accumulate(ident<double>(), maximum<double>());
        Vector3 ctoAdd = Vector3(0.5, 0.5, 0.5) - boundingBox.getCenter() * cscale;
        for(i = 0; i < (int)vertices.size(); ++i) {
            vertices[i].pos = ctoAdd + vertices[i].pos * cscale;
        }
        toAdd = ctoAdd + cscale * toAdd;
        scale *= cscale;
    }

    void Mesh::sortEdges() {
        //TODO: implement for when reading files other than obj
    }

    struct MFace
    {
        MFace(int v1, int v2, int v3) {
            v[0] = v1; v[1] = v2; v[2] = v3;
            std::sort(v, v + 3);
        }

        bool operator<(const MFace &f) const { return std::lexicographical_compare(v, v + 3, f.v, f.v + 3); }
        int v[3];
    };

    void Mesh::fixDupFaces() {
        int i;
        std::map<MFace, int> faces;
        for(i = 0; i < (int)edges.size(); i += 3) {
            MFace current(edges[i].vertex, edges[i + 1].vertex, edges[i + 2].vertex);

            if(faces.count(current)) {
                int oth = faces[current];
                if(oth == -1) {
                    faces[current] = i;
                    continue;
                }
                faces[current] = -1;
                int newOth = edges.size() - 6;
                int newCur = edges.size() - 3;

                edges[oth] = edges[newOth];
                edges[oth + 1] = edges[newOth + 1];
                edges[oth + 2] = edges[newOth + 2];
                edges[i] = edges[newCur];
                edges[i + 1] = edges[newCur + 1];
                edges[i + 2] = edges[newCur + 2];

                MFace newOthF(edges[newOth].vertex, edges[newOth + 1].vertex, edges[newOth + 2].vertex);
                faces[newOthF] = newOth;

                edges.resize(edges.size() - 6);
                i -= 3;
            }
            else {
                faces[current] = i;
            }
        }

        //scan for unreferenced vertices and get rid of them
        std::set<int> referencedVerts;
        for(i = 0; i < (int)edges.size(); ++i) {
            if(edges[i].vertex < 0 || edges[i].vertex >= (int)vertices.size())
                continue;
            referencedVerts.insert(edges[i].vertex);
        }

        std::vector<int> newIdxs(vertices.size(), -1);
        int curIdx = 0;
        for(i = 0; i < (int)vertices.size(); ++i) {
            if(referencedVerts.count(i))
                newIdxs[i] = curIdx++;
        }

        for(i = 0; i < (int)edges.size(); ++i) {
            if(edges[i].vertex < 0 || edges[i].vertex >= (int)vertices.size())
                continue;
            edges[i].vertex = newIdxs[edges[i].vertex];
        }
        for(i = 0; i < (int)vertices.size(); ++i) {
            if(newIdxs[i] > 0)
                vertices[newIdxs[i]] = vertices[i];
        }
        vertices.resize(referencedVerts.size());
    }

    void Mesh::readObj(std::istream &strm) {
        int i;
        int lineNum = 0;
        while (!strm.eof()) {
            ++lineNum;

            std::vector<std::string> words = readWords(strm);

            if(words.size() == 0)
                continue;

            if(words[0][0] == '#')                          //comment
                continue;

            //deal with the line based on the first word
            if (words[0] == "vt") {                         // texture coordinates
                double tx, ty;
                withTexture = true;
                sscanf(words[1].c_str(), "%lf", &tx);
                sscanf(words[2].c_str(), "%lf", &ty);
                vertices.back().texture = Vector3(tx, ty, 0.0);

            }                                               // vertex normal
            else if (words[0] == "vn") {
                if (words.size() != 4) {
                    Debugging::out() << "Error on line " << lineNum << std::endl;
                    OUT;
                }

                double nx, ny, nz;
                sscanf(words[1].c_str(), "%lf", &nx);
                sscanf(words[2].c_str(), "%lf", &ny);
                sscanf(words[3].c_str(), "%lf", &nz);

            }                                               // geometric vertices
            else if(words[0][0] == 'v' && words[0].size() == 1) {
                if (words.size() != 4) {
                    Debugging::out() << "Error on line " << lineNum << std::endl;
                    OUT;
                }

                double x, y, z;
                sscanf(words[1].c_str(), "%lf", &x);
                sscanf(words[2].c_str(), "%lf", &y);
                sscanf(words[3].c_str(), "%lf", &z);

                vertices.resize(vertices.size() + 1);
                vertices.back().pos = Vector3(x, y, z);

            }                                               // unknown line
            else if(words[0].size() != 1) {
                continue;
            }

            if (words[0][0] == 'f') {                       // polygonal face element
                if (words.size() < 4 || words.size() > 15) {
                    Debugging::out() << "Error on line " << lineNum << std::endl;
                    OUT;
                }

                int a[16];
                int t[16];
                for(i = 0; i < (int)words.size() - 1; ++i) {
                    sscanf(words[i + 1].c_str(), "%d/%d", a + i, t + i);
                }

                //swap(a[1], a[2]); //TODO:remove

                for(int j = 2; j < (int)words.size() - 1; ++j) {
                    int first = edges.size();
                    edges.resize(edges.size() + 3);
                    edges[first].vertex = a[0] - 1;
                    edges[first + 1].vertex = a[j - 1] - 1;
                    edges[first + 2].vertex = a[j] - 1;

                    edges[first].tvertex = t[0] - 1;
                    edges[first + 1].tvertex = t[j - 1] - 1;
                    edges[first + 2].tvertex = t[j] - 1;
                }

            }

            //otherwise continue -- unrecognized line
        }
    }

    /*
        //if(words[0].size() != 1) //unknown line
        //  continue;

        //deal with the line based on the first word
        if(words[0][0] == 'v') {
          if(words.size() != 4) {
            Debugging::out() << "Error on line " << lineNum << std::endl;
            OUT;
          }

          double x, y, z;
          sscanf(words[1].c_str(), "%lf", &x);
          sscanf(words[2].c_str(), "%lf", &y);
          sscanf(words[3].c_str(), "%lf", &z);

          vertices.resize(vertices.size() + 1);
          vertices.back().pos = Vector3(x, y, z);
        }

        if(words[0][0] == 'f') {
          if(words.size() < 4 || words.size() > 15) {
            Debugging::out() << "Error on line " << lineNum << std::endl;
            OUT;
          }

          int a[16];
          for(i = 0; i < (int)words.size() - 1; ++i)
            sscanf(words[i + 1].c_str(), "%d", a + i);

          //swap(a[1], a[2]); //TODO:remove

          for(int j = 2; j < (int)words.size() - 1; ++j)
          {
            int first = edges.size();
            edges.resize(edges.size() + 3);
            edges[first].vertex = a[0] - 1;
            edges[first + 1].vertex = a[j - 1] - 1;
            edges[first + 2].vertex = a[j] - 1;
          }
        }

        //otherwise continue -- unrecognized line
      }
    }
    */

    void Mesh::readPly(std::istream &strm) {
        int i;
        int lineNum = 0;

        bool outOfHeader = false;
        int vertsLeft = -1;

        while(!strm.eof()) {
            ++lineNum;

            std::vector<std::string> words = readWords(strm);

            if(words.size() == 0)
                continue;

            if(words[0][0] == '#')                          //comment
                continue;

            if(!outOfHeader) {                              //look for end_header or verts
                if(words[0] == std::string("end_header")) {
                    if(vertsLeft < 0) {
                        Debugging::out() << "Error: no vertex count in header" << std::endl;
                        OUT;
                    }
                    outOfHeader = true;
                    continue;
                }

                if(words.size() < 3)                        //not "element vertex n"
                    continue;

                if(words[0] == std::string("element") && words[1] == std::string("vertex")) {
                    sscanf(words[2].c_str(), "%d", &vertsLeft);
                }

                continue;
            }

            //if there are verts left, current line is a vertex
            if(vertsLeft > 0) {
                --vertsLeft;
                if(words.size() < 3) {
                    Debugging::out() << "Error on line " << lineNum << std::endl;
                    OUT;
                }

                double x, y, z;
                int t1, t2, t3, t4;
                if(words.size() <= 6) {
                    sscanf(words[0].c_str(), "%lf", &x);
                    sscanf(words[1].c_str(), "%lf", &y);
                    sscanf(words[2].c_str(), "%lf", &z);
                    vertices.resize(vertices.size() + 1);
                    vertices.back().pos = Vector3(-z, x, -y);
                }
                else if (words.size() == 7) {
                    withTexture = true;
                    sscanf(words[0].c_str(), "%lf", &x);
                    sscanf(words[1].c_str(), "%lf", &y);
                    sscanf(words[2].c_str(), "%lf", &z);
                    sscanf(words[3].c_str(), "%i", &t1);
                    sscanf(words[4].c_str(), "%i", &t2);
                    sscanf(words[5].c_str(), "%i", &t3);
                    sscanf(words[6].c_str(), "%i", &t4);
                    vertices.resize(vertices.size() + 1);
                    vertices.back().pos = Vector3(-z, x, -y);
                    vertices.back().texture = Vector3(t1, t2, t3);
                }
                continue;
            }

            //otherwise it's a face
            if(words.size() != 4) {
                Debugging::out() << "Error on line " << lineNum << std::endl;
                OUT;
            }

            int a[3];
            for(i = 0; i < 3; ++i) {
                sscanf(words[i + 1].c_str(), "%d", a + i);
            }

            int first = edges.size();
            edges.resize(edges.size() + 3);
            for(i = 0; i < 3; ++i) {
                //indices in file are 0-based
                edges[first + i].vertex = a[i];
            }

            //otherwise continue -- unrecognized line
        }
    }

    void Mesh::readOff(std::istream &strm) {
        int i;
        int lineNum = 0;

        bool outOfHeader = false;
        int vertsLeft = -1;

        while(!strm.eof()) {
            ++lineNum;

            std::vector<std::string> words = readWords(strm);

            if(words.size() == 0)
                continue;

            if(words[0][0] == '#')                          //comment
                continue;

            if(!outOfHeader) {                              //look for number of verts
                if(words.size() < 3)                        //not "vertices faces 0"
                    continue;
                sscanf(words[0].c_str(), "%d", &vertsLeft);
                outOfHeader = true;
                continue;
            }

            //if there are verts left, current line is a vertex
            if(vertsLeft > 0) {
                --vertsLeft;
                if(words.size() < 3) {
                    Debugging::out() << "Error on line " << lineNum << std::endl;
                    OUT;
                }

                double x, y, z;
                sscanf(words[0].c_str(), "%lf", &x);
                sscanf(words[1].c_str(), "%lf", &y);
                sscanf(words[2].c_str(), "%lf", &z);

                vertices.resize(vertices.size() + 1);
                vertices.back().pos = Vector3(x, y, z);

                continue;
            }

            //otherwise it's a face
            if(words.size() != 4) {
                Debugging::out() << "Error on line " << lineNum << std::endl;
                OUT;
            }

            int a[3];
            for(i = 0; i < 3; ++i) {
                sscanf(words[i + 1].c_str(), "%d", a + i);
            }

            int first = edges.size();
            edges.resize(edges.size() + 3);
            for(i = 0; i < 3; ++i) {
                edges[first + i].vertex = a[i];             //indices in file are 0-based
            }

            //otherwise continue -- unrecognized line
        }
    }

    void Mesh::readGts(std::istream &strm) {
        int i;
        int lineNum = 0;

        bool outOfHeader = false;
        int vertsLeft = -1;
        int edgesLeft = -1;

        std::vector<std::pair<int, int> > fedges;

        while(!strm.eof()) {
            ++lineNum;

            std::vector<std::string> words = readWords(strm);

            if(words.size() == 0)
                continue;

            if(words[0][0] == '#')                          //comment
                continue;

            if(!outOfHeader) {                              //look for number of verts
                if(words.size() < 3)                        //not "vertices faces 0"
                    continue;
                sscanf(words[0].c_str(), "%d", &vertsLeft);
                sscanf(words[1].c_str(), "%d", &edgesLeft);
                outOfHeader = true;
                continue;
            }

            if(vertsLeft > 0) {                             //if there are verts left, current line is a vertex
                --vertsLeft;
                if(words.size() < 3) {
                    Debugging::out() << "Error on line " << lineNum << std::endl;
                    OUT;
                }

                double x, y, z;
                sscanf(words[0].c_str(), "%lf", &x);
                sscanf(words[1].c_str(), "%lf", &y);
                sscanf(words[2].c_str(), "%lf", &z);

                vertices.resize(vertices.size() + 1);
                vertices.back().pos = Vector3(-x, z, y);

                continue;
            }

            if(edgesLeft > 0) {
                --edgesLeft;
                if(words.size() != 2) {
                    Debugging::out() << "Error (edge) on line " << lineNum << std::endl;
                    OUT;
                }
                int e1, e2;
                sscanf(words[0].c_str(), "%d", &e1);
                sscanf(words[1].c_str(), "%d", &e2);
                fedges.push_back(std::make_pair(e1 - 1, e2 - 1));
                continue;
            }

            if(words.size() != 3) {                         //otherwise it's a face
                Debugging::out() << "Error on line " << lineNum << std::endl;
                OUT;
            }

            int a[3];
            for(i = 0; i < 3; ++i) {
                sscanf(words[i].c_str(), "%d", a + i);
                --(a[i]);                                   //indices in file are 1-based
            }

            int first = edges.size();
            edges.resize(edges.size() + 3);
            for(i = 0; i < 3; ++i) {
                int ni = (i + 1) % 3;

                if(fedges[a[i]].first == fedges[a[ni]].first) {
                    edges[first + i].vertex = fedges[a[i]].first;
                }
                else if(fedges[a[i]].first == fedges[a[ni]].second) {
                    edges[first + i].vertex = fedges[a[i]].first;
                }
                else if(fedges[a[i]].second == fedges[a[ni]].first) {
                    edges[first + i].vertex = fedges[a[i]].second;
                }
                else if(fedges[a[i]].second == fedges[a[ni]].second) {
                    edges[first + i].vertex = fedges[a[i]].second;
                }
            }

            //otherwise continue -- unrecognized line
        }
    }

    class StlVtx : public Vector3
    {
        public:
            StlVtx(double x, double y, double z) : Vector3(x, y, z) {}
            bool operator==(const StlVtx &o) const { return (*this)[0] == o[0] && (*this)[1] == o[1] && (*this)[2] == o[2]; }
            bool operator<(const StlVtx &o) const
            {
                return (*this)[0] < o[0] || ((*this)[0] == o[0] &&
                    ((*this)[1] < o[1] || ((*this)[1] == o[1] && (*this)[2] < o[2])));
            }
    };

}                                                           // namespace Pinocchio


namespace std
{
    template<> struct hash<Pinocchio::StlVtx>
    {
        std::size_t operator()(const Pinocchio::StlVtx &p) const
        {
            return (int)(p[0] * 100000. + p[1] * 200000. + p[2] * 400000.);
        }
    };
}


namespace Pinocchio {

    void Mesh::readStl(std::istream &strm) {
        int i;
        int lineNum = 0;

        std::unordered_map<StlVtx, int> vertexIdx;

        std::vector<int> lastIdxs;

        Vector3 normal;

        while(!strm.eof()) {
            ++lineNum;

            std::vector<std::string> words = readWords(strm);

            if(words.size() == 0)
                continue;
            //comment
            if(words[0][0] == '#')
                continue;

            if(words[0] == std::string("vertex")) {
                double x, y, z;
                sscanf(words[1].c_str(), "%lf", &x);
                sscanf(words[2].c_str(), "%lf", &y);
                sscanf(words[3].c_str(), "%lf", &z);

                StlVtx cur(y, z, x);
                int idx;

                if(vertexIdx.find(cur) == vertexIdx.end()) {
                    idx = vertices.size();
                    vertexIdx[cur] = idx;
                    vertices.resize(vertices.size() + 1);
                    vertices.back().pos = cur;
                }
                else
                    idx = vertexIdx[cur];

                lastIdxs.push_back(idx);
                if(lastIdxs.size() > 3)
                    lastIdxs.erase(lastIdxs.begin());
                continue;
            }

            if(words[0] == std::string("endfacet")) {
                if(lastIdxs[0] == lastIdxs[1] || lastIdxs[1] == lastIdxs[2] || lastIdxs[0] == lastIdxs[2]) {
                    Debugging::out() << "Duplicate vertex in triangle" << std::endl;
                    continue;
                }
                int first = edges.size();
                edges.resize(edges.size() + 3);
                for(i = 0; i < 3; ++i) {
                    //indices in file are 0-based
                    edges[first + i].vertex = lastIdxs[i];
                }
                continue;
            }

            //otherwise continue -- unrecognized line
        }
    }

    void Mesh::writeObj(const std::string &filename) const
    {
        int i;
        std::ofstream os(filename.c_str());

        for(i = 0; i < (int)vertices.size(); ++i)
            os << "v " << vertices[i].pos[0] << " " << vertices[i].pos[1] << " " << vertices[i].pos[2] << std::endl;

        for(i = 0; i < (int)edges.size(); i += 3)
            os << "f " << edges[i].vertex + 1 << " " << edges[i + 1].vertex + 1 << " " << edges[i + 2].vertex + 1 << std::endl;
    }

    bool Mesh::isConnected() const
    {
        if(vertices.size() == 0)
            return false;

        std::vector<bool> reached(vertices.size(), false);
        std::vector<int> todo(1, 0);
        reached[0] = true;
        unsigned int reachedCount = 1;

        int inTodo = 0;
        while(inTodo < (int)todo.size()) {
            int startEdge = vertices[todo[inTodo++]].edge;
            int curEdge = startEdge;
            do {
                //walk around
                curEdge = edges[edges[curEdge].prev].twin;
                int vtx = edges[curEdge].vertex;
                if(!reached[vtx]) {
                    reached[vtx] = true;
                    ++reachedCount;
                    todo.push_back(vtx);
                }
            } while(curEdge != startEdge);
        }

        return reachedCount == vertices.size();
    }

    #define CHECK(pred) { if(!(pred)) { Debugging::out() << "Mesh integrity error: " #pred << " in line " << __LINE__ << std::endl; return false; } }

    bool Mesh::integrityCheck() const
    {
        int i;
        int vs = vertices.size();
        int es = edges.size();

        //if there are no vertices, shouldn't be any edges either
        if(vs == 0) {
            CHECK(es == 0);
            return true;
        }

        //otherwise, there should be edges
        CHECK(es > 0);

        //check index range validity
        for(i = 0; i < vs; ++i) {
            CHECK(vertices[i].edge >= 0);
            CHECK(vertices[i].edge < es);
        }

        for(i = 0; i < es; ++i) {
            CHECK(edges[i].vertex >= 0 && edges[i].vertex < vs);
            CHECK(edges[i].prev >= 0 && edges[i].prev < es);
            CHECK(edges[i].twin >= 0 && edges[i].twin < es);
        }

        //check basic edge and vertex relationships
        for(i = 0; i < es; ++i) {
            CHECK(edges[i].prev != i);                      //no loops
            CHECK(edges[edges[edges[i].prev].prev].prev == i);//we have only triangles
            CHECK(edges[i].twin != i);                      //no self twins
            CHECK(edges[edges[i].twin].twin == i);          //twins are valid

            //twin's vertex and prev's vertex should be the same
            CHECK(edges[edges[i].twin].vertex == edges[edges[i].prev].vertex);
        }

        for(i = 0; i < vs; ++i) {                           //make sure the edge pointer is correct
            CHECK(edges[edges[vertices[i].edge].prev].vertex == i);
        }

        //check that the edges around a vertex form a cycle -- by counting
        //how many edges adjacent to each vertex
        std::vector<int> edgeCount(vs, 0);
        for(i = 0; i < es; ++i)
            edgeCount[edges[i].vertex] += 1;

        for(i = 0; i < vs; ++i) {
            int startEdge = vertices[i].edge;
            int curEdge = startEdge;
            int count = 0;
            do {                                            //walk around
                curEdge = edges[edges[curEdge].prev].twin;
                ++count;
            } while(curEdge != startEdge && count <= edgeCount[i]);
            CHECK(count == edgeCount[i] && "Non-manifold vertex found");
        }

        return true;
    }

}                                                           // namespace Pinocchio
