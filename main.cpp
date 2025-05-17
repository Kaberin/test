/**
 * @file main.cpp
 * @author Vyacheslav (you@domain.com)
 * @brief Вычисление пересечения отрезков. Функция Intersect возвращает точку если есть пересечения и точку с координанами nan если его нет.
 * @version 0.1
 * @date 16-05-2025
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>

constexpr double eps = 10e-6;

class Vector3D {
    double X;
    double Y;
    double Z;
public:
    double getX() const
    {
        return X;
    }
    double getY() const
    {
        return Y;
    }
    double getZ() const
    {
        return Z;
    }

    Vector3D(double x = 0, double y = 0, double z = 0) : X{ x }, Y{ y }, Z{ z } {};
    double dot_product(const Vector3D& v) const
    {
        return { v.X * X + v.Y * Y + v.Z * Z };
    }

    Vector3D cross_product(const Vector3D& v) const
    {
        Vector3D result{ Y * v.Z - Z * v.Y, -(X * v.Z - Z * v.X), X * v.Y - Y * v.X };
        return result;
    }

    Vector3D operator-(const Vector3D& v) const
    {
        return { X - v.X, Y - v.Y, Z - v.Z };
    }

    Vector3D operator+(const Vector3D& v) const
    {
        return { X + v.X, Y + v.Y, Z + v.Z };
    }
    Vector3D operator*(double factor) const
    {
        return { X * factor, Y * factor, Z * factor };
    }
    bool operator ==(const Vector3D& v) const
    {
        return ((v.X - X) <= eps) && ((v.Y - Y) <= eps) && ((v.Z - Z) <= eps);
    }
    bool operator !=(const Vector3D& v) const
    {
        return !(*this == v);
    }

    double length() const
    {
        double len = sqrt(X * X + Y * Y + Z * Z);
        return len;
    }
    bool is_nan() const
    {
        if (std::isnan(X) || std::isnan(Y) || std::isnan(Z))
        {
            return true;
        }
        return false;
    }
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& v);
};

std::ostream& operator<<(std::ostream& os, const Vector3D& v)
{
    os << "X = " << v.getX() << ", Y = " << v.getY() << ", Z = " << v.getZ();
    return os;
}

class Segment3D {
    Vector3D start;
    Vector3D end;
    friend Vector3D Intersect(Segment3D s1, Segment3D s2);
public:
    Segment3D(Vector3D s, Vector3D e) : start{ s }, end{ e } {}
};

double round_to_n(double value, int n)
{
    double factor = std::pow(10.0, n);
    return std::round(value * factor) / factor;
}
Vector3D Intersect(Segment3D s1, Segment3D s2)
{
    Vector3D P1 = s1.start;
    Vector3D P2 = s1.end;
    Vector3D Q1 = s2.start;
    Vector3D Q2 = s2.end;

    Vector3D u = P2 - P1;
    Vector3D v = Q2 - Q1;
    Vector3D w0 = P1 - Q1;

    double a = u.dot_product(u);
    double b = v.dot_product(u);
    double c = v.dot_product(v);
    double d = u.dot_product(w0);
    double e = v.dot_product(w0);

    if (fabs(v.cross_product(u).length()) < eps)
    {
        // std::cout << "Parallel\n";
        double d = u.cross_product(w0).length() / u.length();
        if (d > 0)
        {
            return Vector3D(NAN, NAN, NAN); //Параллельны, расстояние больше 0 --> Не пересекаются
        } else
        {
            double t0 = (Q1 - P1).dot_product(u) / u.dot_product(u);
            double t1 = (Q2 - P1).dot_product(u) / u.dot_product(u);

            if (t0 > t1)
            {
                std::swap(t0, t1);
            }

            double t_start = std::max(0.0, t0);
            double t_end = std::min(1.0, t1);

            if (t_start > t_end)
            {
                return Vector3D(NAN, NAN, NAN); // Нет перекрытия
            }
            return P1 + u * t_start;
        }
    }

    double demoninator = (a * c - b * b);
    double t = round_to_n((-(d * c) + b * e) / demoninator, 6);
    double s = (e + b * t) / c;



    // << std::setprecision(15) << std::fixed
    // std::cout << "t = " << t << ", s = " << s << '\n';

    Vector3D Pt = P1 + u * t;
    Vector3D Qs = Q1 + v * s;


    // std::cout << "Point of intersection Pt: " << Pt << '\n';
    // std::cout << "Point of intersection Qs: " << Qs << '\n';


    if ((Pt - Qs).length() > eps)
    {
        // std::cout << "Len\n";
        return Vector3D(NAN, NAN, NAN);
    }

    if (t > 1 || t < 0 || s > 1 || s < 0)
    {
        // std::cout << "Bound\n";
        return Vector3D(NAN, NAN, NAN); //Невалидное значение, не пересекаются
    }

    return Pt;
}



void test_intersection()
{
    int test_passed = 0;
    int test_failed = 0;


    std::vector<std::pair<Segment3D, Segment3D>> segments{
        // 1. Прямое пересечение в центре
        {
            {{ 1, 0, 0 }, { -1, 0, 0 }},
            {{ 0, 1, 0 }, { 0, -1, 0 }}
        },
        // 2. Пересекаются по диагонали, центр (0, 0, 0)
        {
            {{ 1, 0, -1 }, { -1, 0, 1 }},
            {{ 0, 1, 0 }, { 0, -1, 0 }}
        },
        // 3. То же, но другая высота
        {
            {{ 1, 0, -1 }, { -1, 0, 1 }},
            {{ 0, 1, 1 }, { 0, -1, -1 }}
        },
        // 4. Не пересекаются, разные z
        {
            {{ 1, 0, 2 }, { -1, 0, 2 }},
            {{ 0, 1, 0 }, { 0, -1, 0 }}
        },
        // 5. Параллельные и лежат в одной плоскости, но не совпадают
        {
            {{ 0, 0, 0 }, { 1, 0, 0 }},
            {{ 0, 1, 0 }, { 1, 1, 0 }}
        },
        // 6. Пересекаются, но за пределами отрезков (на продолжениях)
        {
            {{ 0, 0, 0 }, { 1, 1, 1 }},
            {{ 2, 0, 0 }, { 2, 1, 1 }}
        },
        // 7. Точки одинаковые — совпадают
        {
            {{ 0, 0, 0 }, { 1, 1, 1 }},
            {{ 0, 0, 0 }, { 1, 1, 1 }}
        },
        // 8. Отрезки на одной прямой, но не пересекаются
        {
            {{ 0, 0, 0 }, { 1, 1, 1 }},
            {{ 2, 2, 2 }, { 3, 3, 3 }}
        },
        // 9. Отрезки на одной прямой, и касаются концами
        {
            {{ 0, 0, 0 }, { 1, 1, 1 }},
            {{ 1, 1, 1 }, { 2, 2, 2 }}
        },
        // 10. Пересекаются точно по середине
        {
            {{ -1, -1, 0 }, { 1, 1, 0 }},
            {{ -1, 1, 0 }, { 1, -1, 0 }}
        }
    };

    std::vector<Vector3D> expected_results{
        {0, 0, 0},                // 1
        {0, 0, 0},                // 2
        {0, 0, 0},                // 3
        {NAN, NAN, NAN},          // 4
        {NAN, NAN, NAN},          // 5
        {NAN, NAN, NAN},          // 6 — пересекаются вне отрезков
        {0, 0, 0},                // 7 — совпадают, можно возвращать start
        {NAN, NAN, NAN},          // 8 — не пересекаются
        {1, 1, 1},                // 9 — касание концами
        {0, 0, 0}                 // 10 — диагональное пересечение
    };

    for (int i = 0; i < segments.size(); i++)
    {
        Vector3D result = Intersect(segments[i].first, segments[i].second);
        Vector3D expected = expected_results[i];
        if (result.is_nan() && expected.is_nan())
        {
            test_passed++;
            continue;
        }
        if (result != expected)
        {
            test_failed++;
            std::cout << i << " test failed\n Expected " << expected << " but got " << result << '\n';
        } else
        {
            test_passed++;
        }

    }



    std::cout << "Testing ended. Tests passed: " << test_passed << ", Tests failed: " << test_failed << '\n';
}


int main()
{

    test_intersection();

    return 0;
}