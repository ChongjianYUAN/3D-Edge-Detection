#ifndef ANALYZE_HPP
#define ANALYZE_HPP

struct pose
{
    float tx, ty, tz, w, x, y, z;
    pose(float _tx, float _ty, float _tz, float _w, float _x, float _y, float _z) : tx(_tx), ty(_ty), tz(_tz), w(_w), x(_x), y(_y), z(_z){}
};

struct star_end
{
    int st, ed;
    star_end(int _s, int _e) : st(_s), ed(_e){}
};

#endif