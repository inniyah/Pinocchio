// Regression tests for the bug fixes. Run with "make check" from the top directory.
#include "../Pinocchio/pinocchioApi.h"
#include "../Pinocchio/skeleton.h"
#include "../Pinocchio/debugging.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace Pinocchio;

static int failures = 0;
#define EXPECT(c) do { if(!(c)) { std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << ": " #c << std::endl; ++failures; } } while(0)

static std::string writeTmp(const std::string &name, const std::string &text) {
    std::string path = "/tmp/pinocchio_test_" + name;
    std::ofstream(path.c_str()) << text;
    return path;
}

// Closed tetrahedron, with optional extra lines before the vertices
static std::string tetra(const std::string &before, const std::string &faces) {
    return before + "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n" + faces;
}

// Skeleton built in code, to reach the protected helpers
struct TestSkeleton : public Skeleton {
    TestSkeleton(bool fatOnChain) {
        makeJoint("r", Vector3(0, 0, 0));
        makeJoint("a", Vector3(0, 1, 0), "r");
        makeJoint("b", Vector3(0, 2, 0), "a");
        makeJoint("c", Vector3(0, -1, 0), "r");
        initCompressed();
        if(fatOnChain) setFat("a");   // "a" has degree 2
    }
};

int main() {
    std::ofstream devnull("/dev/null");
    Debugging::setOutStream(devnull);

    // fixDupFaces: unused vertex 0 -> first used vertex must end up at index 0
    {
        Mesh m(writeTmp("unref.obj", tetra("v 9 9 9\n", "f 2 4 3\nf 2 3 5\nf 2 5 4\nf 3 4 5\n")));
        EXPECT(m.vertices.size() == 4 && m.vertices[0].pos == Vector3(0, 0, 0));
        EXPECT(m.integrityCheck());
    }
    // fixDupFaces: duplicate pair at the end of the face list used to corrupt the mesh
    {
        Mesh m(writeTmp("dup.obj", tetra("", "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\nf 1 2 4\nf 4 2 1\n")));
        EXPECT(m.edges.size() == 9 || m.edges.size() == 0);
    }
    // OBJ: negative indices, long polygons, "vt" before "v", per-vertex texture from faces
    {
        Mesh n(writeTmp("neg.obj", tetra("", "f -4 -2 -3\nf -4 -3 -1\nf -4 -1 -2\nf -3 -2 -1\n")));
        EXPECT(n.vertices.size() == 4 && n.integrityCheck());

        std::ostringstream big;
        for(int i = 0; i < 20; ++i) big << "v " << cos(i * 0.314) << " " << sin(i * 0.314) << " 0\n";
        big << "f"; for(int i = 1; i <= 20; ++i) big << " " << i; big << "\n";
        Mesh p(writeTmp("big.obj", big.str()));
        EXPECT(p.edges.size() == 18 * 3);

        Mesh t(writeTmp("tex.obj", tetra("vt 0.25 0.75\nvt 0.5 0.5\n",
                        "f 1/2 3/1 2/1\nf 1/2 2/1 4/1\nf 1/2 4/1 3/1\nf 2/1 3/1 4/1\n")));
        EXPECT(t.vertices.size() == 4 && t.withTexture);
        EXPECT(t.vertices.size() == 4 && t.vertices[0].texture == Vector3(0.5, 0.5, 0));
        EXPECT(t.vertices.size() == 4 && t.vertices[1].texture == Vector3(0.25, 0.75, 0));
    }
    // OFF with a quad face (used to be rejected)
    {
        Mesh m(writeTmp("cube.off", "OFF\n8 6 0\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n0 0 1\n1 0 1\n1 1 1\n0 1 1\n"
                        "4 0 3 2 1\n4 4 5 6 7\n4 0 1 5 4\n4 2 3 7 6\n4 1 2 6 5\n4 0 4 7 3\n"));
        EXPECT(m.vertices.size() == 8 && m.edges.size() == 36 && m.integrityCheck());
    }
    // open mesh: prepareMesh must reject it instead of hanging
    {
        Mesh m(writeTmp("open.obj", tetra("", "f 1 3 2\nf 1 2 4\nf 1 4 3\n")));
        EXPECT(prepareMesh(m).vertices.size() == 0);
    }
    // skeletons
    {
        FileSkeleton root2(writeTmp("root2.skel", "r 0 0 0 -1\na 0 1 0 r\nb 0 -1 0 r\n"));
        EXPECT(root2.cfMap().size() == 3 && root2.cfMap()[0] == 0);   // root with 2 children kept

        bool threw = false;
        try { FileSkeleton s(writeTmp("typo.skel", "r 0 0 0 -1\na 0 1 0 rr\n")); }
        catch(const std::runtime_error &) { threw = true; }
        EXPECT(threw);   // unknown parent

        threw = false;
        try { FileSkeleton s(writeTmp("badnum.skel", "r 0 abc 0 -1\n")); }
        catch(const std::runtime_error &) { threw = true; }
        EXPECT(threw);

        threw = false;
        try { FileSkeleton s("/nonexistent/skeleton.txt"); }
        catch(const std::runtime_error &) { threw = true; }
        EXPECT(threw);

        threw = false;
        try { TestSkeleton s(true); }
        catch(const std::runtime_error &) { threw = true; }
        EXPECT(threw);   // 'fat' on a degree-2 joint used to write cFatV[-1]
    }
    // full rig + DQS with uniform scale must match LBS; every vertex has weights
    {
        Mesh m("../models/human_male.obj");
        EXPECT(m.vertices.size() > 0);
        m.normalizeBoundingBox();
        m.computeVertexNormals();
        Skeleton sk = HumanSkeleton();
        sk.scale(0.7);
        PinocchioOutput o = autorig(sk, m);
        EXPECT(o.embedding.size() == sk.fPrev().size() && o.attachment);
        if(o.attachment) {
            std::vector<Transform<> > T(sk.fPrev().size() - 1,
                Transform<>(Quaternion<>(Vector3(0, 1, 0), 0.7), 1.5, Vector3(0.1, 0.2, 0.3)));
            Mesh lbs = o.attachment->deform(m, T, LBS);
            Mesh dqs = o.attachment->deform(m, T, DQS);
            double maxDiff = 0;
            for(int i = 0; i < (int)m.vertices.size(); ++i)
                maxDiff = std::max(maxDiff, (lbs.vertices[i].pos - dqs.vertices[i].pos).length());
            EXPECT(maxDiff < 1e-4);   // DQS used to ignore the 1.5 scale
            double err = 0;
            for(int i = 0; i < (int)m.vertices.size(); ++i) {
                Vector<double, -1> w = o.attachment->getWeights(i);
                double s = 0; for(int j = 0; j < w.size(); ++j) s += w[j];
                err += fabs(s - 1.);
            }
            EXPECT(err < 1e-6);
            delete o.attachment;
        }
    }
    std::cout << (failures ? "FAILED: " : "OK: all tests passed") << (failures ? std::to_string(failures) : "") << std::endl;
    return failures ? 1 : 0;
}
