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

#ifndef MATHUTILS_H_2686C802_462F_11E9_BF34_F7BC72BBEB09
#define MATHUTILS_H_2686C802_462F_11E9_BF34_F7BC72BBEB09

#include <math.h>
#include <functional>
#include <string>
#include "Pinocchio.h"

#ifdef _WIN32
#pragma warning(disable:4355)
#pragma warning(disable:4996)
#pragma warning(disable:4251)
//_WIN32
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline int ROUND(double x) { return (int)(x + 0.5); }
inline int SIGN(double x) { return (x > 0.) ? 1 : -1; }
template<class T> T SQR(const T & x) { return x * x; }
template<class T> T CUBE(const T & x) { return x * x * x; }
template<class T> T QUAD(const T & x) { return SQR(SQR(x)); }

template <class T>
class maximum {
    public:
        typedef T first_argument_type;
        typedef T second_argument_type;
        typedef T result_type;
        T operator()(const T &a1, const T &a2) const { return std::max(a1, a2); }
};

template <class T>
class minimum {
    public:
        typedef T first_argument_type;
        typedef T second_argument_type;
        typedef T result_type;
        T operator()(const T &a1, const T &a2) const { return std::min(a1, a2); }
};

template <class A1, class A2, class R>
class myMult {
    public:
        typedef A1 first_argument_type;
        typedef A2 second_argument_type;
        typedef R result_type;
        R operator()(const A1 &a1, const A2 &a2) const { return a1 * a2; }
};

template <class A1, class A2, class R>
class myDiv {
    public:
        typedef A1 first_argument_type;
        typedef A2 second_argument_type;
        typedef R result_type;
        R operator()(const A1 &a1, const A2 &a2) const { return a1 / a2; }
};

// Replacements for std::bind2nd(op, x), deprecated in C++11 and removed in C++17.
// They expose result_type because Vector::apply needs it, without relying on the
// typedefs of std::plus & co. (also removed in C++20).
template <class Op> struct opResult { typedef typename Op::result_type type; };
template <class R> struct opResult<std::plus<R> > { typedef R type; };
template <class R> struct opResult<std::minus<R> > { typedef R type; };
template <class R> struct opResult<std::multiplies<R> > { typedef R type; };
template <class R> struct opResult<std::divides<R> > { typedef R type; };

template <class Op, class T>
class bindSecond {
    public:
        typedef typename opResult<Op>::type result_type;
        bindSecond(const Op &op, const T &v) : op_(op), v_(v) {}
        template <class A> result_type operator()(const A &a) const { return op_(a, v_); }
    private:
        Op op_;
        T v_;
};

template <class Op, class T>
bindSecond<Op, T> bind2ndCompat(const Op &op, const T &v) { return bindSecond<Op, T>(op, v); }

template <class T>
class ident {
    public:
        typedef T argument_type;
        typedef T result_type;
        T operator()(const T &v) const { return v; }
};

template<class T> class construct {
    public:
        typedef T result_type;
        template<class From> T operator()(const From &x) const { return T(x); }
};

#ifdef _WIN32
inline bool operator==(const string &s1, const string &s2) { return s1.compare(s2) == 0; }
inline bool operator<(const string &s1, const string &s2) { return s1.compare(s2) < 0; }
#endif
#endif // MATHUTILS_H_2686C802_462F_11E9_BF34_F7BC72BBEB09
