#ifndef WINVER
#define WINVER 0x0500
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT WINVER
#endif
#define NOMINMAX
#include <graphics.h>
#include <conio.h>
#include <math.h>
#include <cstring>
#include <cmath>
#include <tchar.h>
#include <windows.h>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <cwchar>
#include <vector>
#include <random>
#include <stdexcept>
#include <limits>
#include <cerrno>
using namespace std;
template <typename... Args>
int formatText(TCHAR* buffer, size_t capacity, const TCHAR* format, Args... args)
{
#ifdef _UNICODE
    int result = std::swprintf(buffer, capacity, format, args...);
#else
    int result = std::snprintf(buffer, capacity, format, args...);
#endif
    if (capacity) buffer[capacity - 1] = 0;
    return result;
}


// ============================================================================
//  山区滚石模拟（米制版）
//  ---------------------------------------------------------------
//  物理量全部用国际单位制：位置 m、速度 m/s、时间 s、质量 kg、能量 J
//  只有在“画图”这一步才换算成像素，换算比例 PX_PER_M
//  刚体模型：均匀凸多边形沿屏幕法向挤出 1m；质量=rho*S*1m。
//  使用真实质心、极惯性矩、接触点冲量、库仑摩擦和恢复系数。
//  地形是静态折线；这是二维数值近似，不包括破碎、三维翻滚和材料变形。
//  接触求解采用固定步长、子步和迭代冲量；参考 https://box2d.org/documentation/md_simulation.html
// ============================================================================

// ------------------------------------------------------------------ 画面
#define WIDTH 1000	   // 窗口宽度（像素）
#define HEIGHT 600	   // 窗口高度（像素）
#define PX_PER_M 8.0   // 显示比例：1 米 = 8 像素（窗口即 125 m × 75 m）
#define ORIGIN_Y 490.0 // 高程 z=0 对应的屏幕 y 坐标
#define GROUND_Y 490   // 坡脚基准线（屏幕 y）

// ------------------------------------------------------------------ 物理常数
#define GRAVITY 9.81		// 重力加速度 (m/s²)
#define DT 0.002			// 物理步长 (s)
#define STEPS_PER_FRAME 8	// 每帧 8 步 × 2 ms = 16 ms ≈ 实时
#define ROCK_DENSITY 2600.0 // 岩石密度 (kg/m³)
#define AIR_DENSITY 1.225	// 空气密度 (kg/m³)
#define MIN_RADIUS 0.2		// 半径可调下限 (m)，只用来防止误操作，不是默认半径
#define MAX_RADIUS 3.0		// 半径可调上限 (m)
#define BOUNCE_MIN 0.15		// 法向速度小于此值不再弹跳，按接触处理 (m/s)
#define REST_SPEED 0.05		// 判定“停住”的速度阈值 (m/s)

// 可调参数（程序运行时通过菜单修改）
double g_rollResist = 0.02;	   // 滚动阻力系数 c_rr（0.02~0.20）
double g_slideFriction = 0.45; // 地面摩擦系数 μ（tan 24°，坡面摩擦角约 24°）
double g_restitution = 0.45;   // 法向恢复系数 e
double g_dragCoef = 1.0;	   // 空气阻力系数 Cd（0 表示真空）
int g_playbackFactor = 1;	   // 1x,2x,4x,8x

// ------------------------------------------------------------------ 坐标换算
double toPxX(double x) { return x * PX_PER_M; }
double toPxY(double z) { return ORIGIN_Y - z * PX_PER_M; }

// 地形点（x 水平距离 m，z 高程 m，向上为正）
struct Point
{
	double x, z;
	Point(double x = 0, double z = 0) : x(x), z(z) {}
};

// 时间-速度数据结构（替代 std::pair<double,double>）
struct TimeSpeed
{
	double t;
	double s;
	TimeSpeed(double tt = 0.0, double ss = 0.0) : t(tt), s(ss) {}
};

// 简单动态数组模板（替代 std::vector）
template <typename T>
class DynamicArray
{
private:
	T *data;
	size_t cnt;
	size_t cap;

	void grow()
	{
		size_t newCap = cap == 0 ? 4 : cap * 2;
		T *newData = new T[newCap];
		if (data)
		{
			for (size_t i = 0; i < cnt; ++i)
				newData[i] = data[i];
			delete[] data;
		}
		data = newData;
		cap = newCap;
	}

public:
	DynamicArray() : data(nullptr), cnt(0), cap(0) {}
	~DynamicArray() { delete[] data; }
	DynamicArray(const DynamicArray &other) : data(nullptr), cnt(0), cap(0)
	{
		if (other.cnt > 0)
		{
			data = new T[other.cnt];
			for (size_t i = 0; i < other.cnt; ++i)
				data[i] = other.data[i];
			cnt = other.cnt;
			cap = other.cnt;
		}
	}
	DynamicArray &operator=(const DynamicArray &other)
	{
		if (this == &other)
			return *this;
		delete[] data;
		data = nullptr;
		cnt = 0;
		cap = 0;
		if (other.cnt > 0)
		{
			data = new T[other.cnt];
			for (size_t i = 0; i < other.cnt; ++i)
				data[i] = other.data[i];
			cnt = other.cnt;
			cap = other.cnt;
		}
		return *this;
	}

	void push_back(const T &v)
	{
		if (cnt >= cap)
			grow();
		data[cnt++] = v;
	}

	void erase_front()
	{
		if (cnt == 0)
			return;
		for (size_t i = 1; i < cnt; ++i)
			data[i - 1] = data[i];
		--cnt;
	}

	size_t size() const { return cnt; }
	bool empty() const { return cnt == 0; }
	T &operator[](size_t i) { return data[i]; }
	const T &operator[](size_t i) const { return data[i]; }
	void clear() { cnt = 0; }
};

// ------------------------------------------------------------------ 山坡地形
// 13 个控制点，x 从 0 到 120 m，高程从 55.6 m 降到 0 m（坡脚）
class HillSlope
{
private:
	DynamicArray<Point> points; // 地形控制点（米）
	int startX, endX;			// 只用于快速判断，单位仍是米（取整）

public:
	HillSlope()
	{
		// 一条长 120 m、落差 55.6 m 的山区坡面：上部陡、中部有缓台、坡脚有一段平台
		points.push_back(Point(0.0, 55.6)); // 坡顶（危岩位置）
		points.push_back(Point(10.0, 51.9));
		points.push_back(Point(11.25, 50.6));
		points.push_back(Point(30.0, 35.6)); // 陡坡段
		points.push_back(Point(40.0, 34.4)); // 缓台（约 7°）
		points.push_back(Point(50.0, 23.1)); // 陡坡段
		points.push_back(Point(60.0, 20.6));
		points.push_back(Point(70.0, 15.6));
		points.push_back(Point(80.0, 10.0));
		points.push_back(Point(90.0, 8.75)); // 缓台
		points.push_back(Point(100.0, 2.88));
		points.push_back(Point(120.0, 0.0)); // 坡脚平台（道路位置）

		startX = (int)points[0].x;
		endX = (int)points[points.size() - 1].x;
	}

	// 指定 x 处的地面高程 z (m)
	double getHeight(double x) const
	{
		if (x <= points[0].x)
			return points[0].z;
		if (x >= points[points.size() - 1].x)
			return points[points.size() - 1].z;

		for (size_t i = 0; i < points.size() - 1; i++)
		{
			if (x >= points[i].x && x <= points[i + 1].x)
			{
				double t = (x - points[i].x) / (points[i + 1].x - points[i].x);
				return points[i].z + t * (points[i + 1].z - points[i].z);
			}
		}
		return points[0].z;
	}

	// 指定 x 处的坡角 θ（弧度，>0 表示顺坡向下）
	double getSlopeAngle(double x) const
	{
        if(x < points[0].x) return 0; // 与地形外侧的水平延长线一致
        if(x >= points[points.size()-1].x) return 0;

		for (size_t i = 0; i < points.size() - 1; i++)
		{
			if (x >= points[i].x && x <= points[i + 1].x)
			{
				double dz = points[i + 1].z - points[i].z;
				double dx = points[i + 1].x - points[i].x;
				return -atan2(dz, dx);
			}
		}
		return 0;
	}

	// 绘制山坡（唯一需要把米换成像素的地方）
	void draw() const
	{
		for (size_t i = 0; i < points.size() - 1; i++)
		{
			int x1 = (int)toPxX(points[i].x);
			int y1 = (int)toPxY(points[i].z);
			int x2 = (int)toPxX(points[i + 1].x);
			int y2 = (int)toPxY(points[i + 1].z);

			setcolor(RGB(34, 139, 34));
			setlinestyle(PS_SOLID, 3);
			line(x1, y1, x2, y2);

			setfillcolor(RGB(139, 90, 43));
			int pts[8] = {x1, y1, x2, y2, x2, HEIGHT, x1, HEIGHT};
			fillpoly(4, pts);
		}

		// 坡脚基准线（z = 0）
		setcolor(RGB(160, 82, 45));
		setlinestyle(PS_SOLID, 1);
		line(0, (int)toPxY(0), WIDTH, (int)toPxY(0));
	}

	int getStartX() const { return startX; }
	int getEndX() const { return endX; }
    const DynamicArray<Point>& getPoints() const { return points; }
    // 用于同一刚体求解器的水平面/折线回归验证，也便于以后替换地形。
    explicit HillSlope(const DynamicArray<Point>& terrain) : points(terrain)
    {
        if (points.size() < 2) throw std::runtime_error("Terrain needs two points");
        for (size_t i = 1; i < points.size(); ++i)
            if (points[i].x <= points[i - 1].x) throw std::runtime_error("Terrain x must increase");
        startX = (int)points[0].x; endX = (int)points[points.size() - 1].x;
    }

	int getPeakX() const
	{
		if (points.empty())
			return startX;
		double bestZ = points[0].z;
		int bestX = (int)points[0].x;
		for (size_t i = 1; i < points.size(); ++i)
		{
			if (points[i].z > bestZ)
			{ // 高程最大者即坡顶
				bestZ = points[i].z;
				bestX = (int)points[i].x;
			}
		}
		return bestX;
	}

	int getValleyX() const
	{
		if (points.empty())
			return startX;
		double lowZ = points[0].z;
		int lowX = (int)points[0].x;
		for (size_t i = 1; i < points.size(); ++i)
		{
			if (points[i].z < lowZ)
			{ // 高程最小者即坡脚
				lowZ = points[i].z;
				lowX = (int)points[i].x;
			}
		}
		return lowX;
	}
};

// ------------------------------------------------------------------ 定面积圆内接凸多边形
const double POLY_PI = 3.14159265358979323846;
double cross2(Point a, Point b) { return a.x * b.z - a.z * b.x; }
double dot2(Point a, Point b) { return a.x * b.x + a.z * b.z; }
Point plus2(Point a, Point b) { return Point(a.x + b.x, a.z + b.z); }
Point minus2(Point a, Point b) { return Point(a.x - b.x, a.z - b.z); }
Point times2(Point a, double t) { return Point(a.x * t, a.z * t); }
Point rotate2(Point p, double a) { return Point(cos(a)*p.x-sin(a)*p.z, sin(a)*p.x+cos(a)*p.z); }
struct ShapeSettings { int kMin, kMax; double r, area; };
double maxPolygonArea(int k, double r) { return k * r * r * sin(2 * POLY_PI / k) / 2; }
void validateShape(const ShapeSettings& c)
{
    if (c.kMin < 3 || c.kMax < c.kMin || c.kMax > 30)
        throw std::runtime_error("Side range must satisfy 3 <= min <= max <= 30.");
    if (!std::isfinite(c.r) || c.r < 0.05 || c.r > 10)
        throw std::runtime_error("Radius must be between 0.05 and 10 metres.");
    if (!std::isfinite(c.area) || c.area <= 0 || c.area > maxPolygonArea(c.kMin, c.r) + 1e-12*c.r*c.r)
        throw std::runtime_error("Area must be positive and <= kMin*r*r*sin(2*pi/kMin)/2.");
    // 极薄的形状会使接触求解病态，不把近似退化多边形伪装成可用刚体。
    if (c.area < 1e-4 * c.r * c.r)
        throw std::runtime_error("Area is too small for stable rigid-body simulation (S/r^2 >= 0.0001).");
}
struct PolygonShape
{
    std::vector<Point> local; // 以面积质心为原点，逆时针排列
    Point circleCenter;       // 原生成圆的圆心在质心坐标系下的位置
    double radius, area, mass, inertia, extent;
    PolygonShape(const std::vector<Point>& onCircle, double r) : radius(r)
    {
        // 先平移至顶点均值附近，提高小面积多边形的数值精度。
        Point origin;
        for (size_t i=0; i<onCircle.size(); ++i) origin=plus2(origin,onCircle[i]);
        origin=times2(origin,1.0/onCircle.size());
        double twice=0; Point numerator;
        for(size_t i=0;i<onCircle.size();++i) {
            Point a=minus2(onCircle[i],origin), b=minus2(onCircle[(i+1)%onCircle.size()],origin);
            double cr=cross2(a,b); twice+=cr; numerator=plus2(numerator,times2(plus2(a,b),cr));
        }
        if(twice<=0 || !std::isfinite(twice)) throw std::runtime_error("Degenerate polygon");
        area=twice/2;
        Point centroid=plus2(origin,times2(numerator,1/(3*twice)));
        circleCenter=times2(centroid,-1); extent=0;
        for(size_t i=0;i<onCircle.size();++i) {
            local.push_back(minus2(onCircle[i],centroid));
            extent=max(extent,sqrt(dot2(local.back(),local.back())));
        }
        double polar=0;
        for(size_t i=0;i<local.size();++i) {
            Point a=local[i], b=local[(i+1)%local.size()];
            polar+=cross2(a,b)*(dot2(a,a)+dot2(a,b)+dot2(b,b))/12;
        }
        mass=ROCK_DENSITY*area; // 均匀棱柱，厚度固定 1m
        inertia=ROCK_DENSITY*polar;
        if(inertia<=0 || !std::isfinite(inertia)) throw std::runtime_error("Invalid inertia");
    }
};
double randomUnit(std::mt19937& rng) { return (rng()+0.5)/4294967296.0; }
PolygonShape generatePolygon(int k, double r, double target, std::mt19937& rng)
{
    validateShape(ShapeSettings{k,k,r,target});
    const double maximum=maxPolygonArea(k,r);
    if(fabs(target-maximum)<=1e-12*r*r) {
        // 最大面积只能是正 k 边形；独立随机点恰好命中的概率为零。
        double rotation=randomUnit(rng)*2*POLY_PI; std::vector<Point> p;
        for(int i=0;i<k;++i) { double a=rotation+2*POLY_PI*i/k; p.push_back(Point(r*cos(a),r*sin(a))); }
        return PolygonShape(p,r);
    }
    // 定面积条件采样，不是独立均匀圆周采样。随机改变前 k-1 点的角间隔，
    // 根据面积确定它们所占圆弧的跨度；第 k 点仍由面积方程解析求解。
    // 直接全圆独立采样会使“小面积+多顶点”的接受率趋近零。
    const int m=k-1;
    const double spacing=2*POLY_PI/k;
    const double referenceSpan=(k-2)*spacing;
    std::vector<double> jitter(m,0), fractions(m);
    for(int i=1;i<m-1;++i) jitter[i]=(randomUnit(rng)-0.5)*0.8*spacing;
    auto baseArea = [&](double span) {
        double sum=0;
        for(int i=0;i<m-1;++i) sum+=sin(span*(fractions[i+1]-fractions[i]));
        return r*r*(sum-sin(span))/2;
    };
    // 在首尾之间的闭合圆弧内添加一点，面积增加量的上限（插入弧中点）。
    auto maxAddition = [&](double span) {
        return r*r*sin(span/2)*(1+cos(span/2));
    };
    double refBase=0,refAddition=maxAddition(referenceSpan);
    double amplitude=1;
    bool feasible=false;
    for(int iteration=0;iteration<60;++iteration) {
        for(int i=0;i<m;++i) fractions[i]=(i*spacing+amplitude*jitter[i])/referenceSpan;
        refBase=baseArea(referenceSpan);
        if(refBase+refAddition>=target) { feasible=true; break; }
        // 接近最大面积时可行形状接近正多边形，收窄随机角间隔扰动。
        amplitude*=0.5;
    }
    if(!feasible) throw std::runtime_error("Area construction failed at floating-point precision.");
    // q 是第 k 点提供的面积增量占当前圆弧最大增量的比例。
    // 在参考跨度处确保总面积 >= S，零跨度处面积为0，因而始终有根。
    double qMin=max(0.05,(target-refBase)/refAddition);
    qMin=min(1.0,qMin);
    double q=qMin+(1-qMin)*(0.1+0.8*randomUnit(rng));
    double lo=0,hi=referenceSpan;
    for(int iteration=0;iteration<80;++iteration) {
        double mid=(lo+hi)/2;
        if(baseArea(mid)+q*maxAddition(mid)<target) lo=mid;
        else hi=mid;
    }
    double span=(lo+hi)/2;
    double halfGap=POLY_PI-span/2;
    // 使用实际剩余面积求第 k 点，避免二分误差累积到面积约束。
    double cosine=cos(halfGap)+(target-baseArea(span))/(r*r*sin(halfGap));
    cosine=max(-1.0,min(1.0,cosine));
    double offset=acos(cosine), midpoint=(span+2*POLY_PI)/2;
    double last=midpoint+((rng()&1)?offset:-offset);
    if(last<=span || last>=2*POLY_PI) throw std::runtime_error("Degenerate final vertex.");
    double rotation=randomUnit(rng)*2*POLY_PI;
    std::vector<Point> p;
    for(int i=0;i<m;++i) {
        double angle=rotation+span*fractions[i];
        p.push_back(Point(r*cos(angle),r*sin(angle)));
    }
    p.push_back(Point(r*cos(rotation+last),r*sin(rotation+last)));
    PolygonShape shape(p,r);
    if(fabs(shape.area-target)>1e-9*r*r) throw std::runtime_error("Area verification failed.");
    return shape;
}
PolygonShape generatePolygon(const ShapeSettings& c, std::mt19937& rng)
{
    validateShape(c);
    std::uniform_int_distribution<int> sides(c.kMin,c.kMax);
    return generatePolygon(sides(rng),c.r,c.area,rng);
}

// ------------------------------------------------------------------ 参数化地形
struct TerrainSettings      //地形参数：用几个数字描述一条山坡
{
    double topHeight = 55.6;      //坡顶高度（m）
    double steepDeg = 40.0;      //陡坡段坡角（度）
    double benchStart = 25.0;      //缓台起点离坡顶的水平距离（m）
    double benchLength = 15.0;      //缓台水平长度（m），0 表示没有缓台
    double benchDeg = 7.0;      //缓台坡角（度）
    double toeLength = 20.0;      //坡脚平台长度（m）
};

const double TERRAIN_MAX_LENGTH = 120.0;      //地形最大水平长度（m），窗口宽 125 m

DynamicArray<Point> buildTerrain(const TerrainSettings& t)      //按地形参数生成山坡折线的控制点
{
    if (!std::isfinite(t.topHeight) || t.topHeight < 5 || t.topHeight > 60)
    {
        throw std::runtime_error("Top height must be between 5 and 60 m.");
    }

    if (!std::isfinite(t.steepDeg) || t.steepDeg < 15 || t.steepDeg > 70)
    {
        throw std::runtime_error("Steep slope angle must be between 15 and 70 degrees.");
    }

    if (!std::isfinite(t.benchDeg) || t.benchDeg < 0 || t.benchDeg >= t.steepDeg)
    {
        throw std::runtime_error("Bench angle must be >= 0 and smaller than the steep slope angle.");
    }

    if (!std::isfinite(t.benchStart) || !std::isfinite(t.benchLength) || !std::isfinite(t.toeLength)
        || t.benchStart < 0 || t.benchLength < 0 || t.toeLength < 5)
    {
        throw std::runtime_error("Bench start/length must be >= 0 and toe length must be >= 5 m.");
    }

    double steepTan = tan(t.steepDeg * POLY_PI / 180);
    double benchTan = tan(t.benchDeg * POLY_PI / 180);

    double benchStartZ = t.topHeight - t.benchStart * steepTan;
    double benchEndX = t.benchStart + t.benchLength;
    double benchEndZ = benchStartZ - t.benchLength * benchTan;

    if (benchEndZ < 1)
    {
        throw std::runtime_error("The bench must end at least 1 m above the toe; move it up or shorten it.");
    }

    double footX = benchEndX + benchEndZ / steepTan;
    double endX = footX + t.toeLength;

    if (endX > TERRAIN_MAX_LENGTH)
    {
        throw std::runtime_error("Terrain is longer than 120 m; use a steeper slope or shorter segments.");
    }

    DynamicArray<Point> p;
    p.push_back(Point(0.0, t.topHeight));

    if (t.benchStart > 0)
    {
        p.push_back(Point(t.benchStart, benchStartZ));
    }

    if (t.benchLength > 0)
    {
        p.push_back(Point(benchEndX, benchEndZ));
    }

    p.push_back(Point(footX, 0.0));
    p.push_back(Point(endX, 0.0));

    return p;
}

// ------------------------------------------------------------------ 多边形刚体与静态折线的接触
class RollingStone
{
    PolygonShape shape;
    double x,z,vx,vz,angle,omega,quietTime;
    bool moving;
    DynamicArray<Point> trail;
    struct Contact { Point arm, normal; double depth, normalSum, tangentSum, bounce; };
    std::vector<Point> vertices() const {
        std::vector<Point> v;
        for(size_t i=0;i<shape.local.size();++i) v.push_back(plus2(Point(x,z),rotate2(shape.local[i],angle)));
        return v;
    }
    std::vector<Contact> contacts(const HillSlope& hill, double margin) const {
        std::vector<Contact> out; std::vector<Point> v=vertices();
        for(size_t i=0;i<v.size();++i) {
            double th=hill.getSlopeAngle(v[i].x); Point n(sin(th),cos(th));
            double d=(v[i].z-hill.getHeight(v[i].x))*n.z;
            if(d<=margin) out.push_back(Contact{minus2(v[i],Point(x,z)),n,-d,0,0,0});
        }
        // 地形凸角可能从两个顶点之间穿入：同时检查多边形下边缘与地形控制点。
        const DynamicArray<Point>& ground=hill.getPoints();
        for(size_t i=0;i<v.size();++i) {
            Point a=v[i], b=v[(i+1)%v.size()], edge=minus2(b,a);
            if(edge.x<=1e-10) continue; // CCW 多边形朝上的内法线，仅取下边缘
            double len=sqrt(dot2(edge,edge)); Point n(-edge.z/len,edge.x/len);
            for(size_t j=0;j<ground.size();++j) {
                if(ground[j].x<=a.x+1e-8 || ground[j].x>=b.x-1e-8) continue;
                double t=(ground[j].x-a.x)/edge.x;
                Point p=plus2(a,times2(edge,t));
                double d=dot2(minus2(p,ground[j]),n);
                if(d<=margin) out.push_back(Contact{minus2(p,Point(x,z)),n,-d,0,0,0});
            }
        }
        return out;
    }
    double contactVelocity(const Contact& c, Point direction) const {
        return dot2(Point(vx-omega*c.arm.z,vz+omega*c.arm.x),direction);
    }
    double effectiveMass(const Contact& c, Point direction) const {
        double lever=cross2(c.arm,direction);
        return 1/shape.mass+lever*lever/shape.inertia;
    }
    void impulse(const Contact& c, Point direction, double amount) {
        vx+=direction.x*amount/shape.mass; vz+=direction.z*amount/shape.mass;
        omega+=cross2(c.arm,direction)*amount/shape.inertia;
    }
    void substep(const HillSlope& hill, double h) {
        vz-=GRAVITY*h;
        double speed=sqrt(vx*vx+vz*vz);
        // 平动空气阻力：迎风宽度近似 2r，棱柱厚度 1m，隐式衰减防止反向。
        double damping=1+0.5*AIR_DENSITY*g_dragCoef*(2*shape.radius)*speed*h/shape.mass;
        vx/=damping; vz/=damping;
        x+=vx*h; z+=vz*h; angle=std::remainder(angle+omega*h,2*POLY_PI);
        std::vector<Contact> c=contacts(hill,0.0002);
        for(size_t i=0;i<c.size();++i) {
            double vn=contactVelocity(c[i],c[i].normal);
            c[i].bounce=vn< -0.5 ? -g_restitution*vn : 0;
        }
        for(int iteration=0;iteration<20;++iteration) {
            for(size_t i=0;i<c.size();++i) {
                Contact& q=c[i];
                double old=q.normalSum;
                q.normalSum=max(0.0,old+(q.bounce-contactVelocity(q,q.normal))/effectiveMass(q,q.normal));
                impulse(q,q.normal,q.normalSum-old);
                Point tangent(q.normal.z,-q.normal.x);
                double limit=g_slideFriction*q.normalSum;
                old=q.tangentSum;
                q.tangentSum=max(-limit,min(limit,old-contactVelocity(q,tangent)/effectiveMass(q,tangent)));
                impulse(q,tangent,q.tangentSum-old);
            }
        }
        double support=0;
        for(size_t i=0;i<c.size();++i) support+=c[i].normalSum;
        double resistance=g_rollResist*shape.radius*support/shape.inertia;
        omega-=max(-resistance,min(resistance,omega));
        // 分离位置修正不改变速度，避免把穿透修正变成额外弹跳能量。
        for(int iteration=0;iteration<12;++iteration) {
            std::vector<Contact> correction=contacts(hill,0);
            double worst=0;
            for(size_t i=0;i<correction.size();++i) {
                Contact& q=correction[i]; worst=max(worst,q.depth);
                if(q.depth<=0.00005) continue;
                double amount=min(0.02,0.7*(q.depth-0.00005))/effectiveMass(q,q.normal);
                x+=q.normal.x*amount/shape.mass; z+=q.normal.z*amount/shape.mass;
                angle+=cross2(q.arm,q.normal)*amount/shape.inertia;
            }
            if(worst<=0.0001) break;
        }
        if(!std::isfinite(x+z+vx+vz+angle+omega)) throw std::runtime_error("Non-finite rigid-body state");
        // 不能仅凭质心低速判断停止：还需接触支撑及转动足够慢，持续 2 秒。
        if(!c.empty() && getSpeed()<0.015 && fabs(omega)*shape.extent<0.015) quietTime+=h;
        else quietTime=0;
        if(quietTime>=2) { moving=false; vx=vz=omega=0; }
    }
public:
    RollingStone(double startX, double startZ, const PolygonShape& p)
        : shape(p),x(startX),z(startZ),vx(0),vz(0),angle(0),omega(0),quietTime(0),moving(true) {}
    void update(const HillSlope& hill, double dt) {
        if(!moving) return;
        // 运动越快/刚体越薄，子步越细；不改变外部统计/显示时间步长。
        double scale=max(0.0005,min(shape.radius*0.1,shape.area/(4*shape.radius)));
        int n=max(2,(int)ceil((getSpeed()+fabs(omega)*shape.extent)*dt/scale));
        if(n>256) throw std::runtime_error("Motion requires too many collision substeps; choose a less thin shape.");
        for(int i=0;i<n && moving;++i) substep(hill,dt/n);
        trail.push_back(Point(x,z)); if(trail.size()>80) trail.erase_front();
    }
    void draw() const {
        setcolor(RGB(210,80,30));
        for(size_t i=1;i<trail.size();++i) line((int)toPxX(trail[i-1].x),(int)toPxY(trail[i-1].z),(int)toPxX(trail[i].x),(int)toPxY(trail[i].z));
        std::vector<Point> v=vertices(); std::vector<int> xy;
        for(size_t i=0;i<v.size();++i) { xy.push_back((int)toPxX(v[i].x)); xy.push_back((int)toPxY(v[i].z)); }
        setfillcolor(RGB(105,110,125)); setcolor(RGB(30,35,50)); setlinestyle(PS_SOLID,2);
        fillpoly((int)v.size(),xy.data());
        setlinestyle(PS_SOLID,1); setfillcolor(RGB(255,220,40));
        solidcircle((int)toPxX(x),(int)toPxY(z),2); // 黄色标记：质心，而非生成圆圆心
    }
    void adjustToGround(const HillSlope& hill) {
        std::vector<Point> v=vertices(); double lift=-1e100;
        for(size_t i=0;i<v.size();++i) lift=max(lift,hill.getHeight(v[i].x)-v[i].z);
        const DynamicArray<Point>& ground=hill.getPoints();
        for(size_t i=0;i<v.size();++i) {
            Point a=v[i],b=v[(i+1)%v.size()]; if(b.x<=a.x) continue;
            for(size_t j=0;j<ground.size();++j) if(ground[j].x>a.x && ground[j].x<b.x)
                lift=max(lift,ground[j].z-(a.z+(b.z-a.z)*(ground[j].x-a.x)/(b.x-a.x)));
        }
        z+=lift+0.0001; vz=0;
    }
    void startAlongSlope(const HillSlope& hill,double speed) {
        double th=hill.getSlopeAngle(x); vx=speed*cos(th); vz=-speed*sin(th);
        omega=0; // 初始无自旋；转动由实际接触点冲量产生，不套用球体 omega=v/r。
        moving=true; quietTime=0;
    }
    bool isMoving() const { return moving; }
    double getX() const { return x; } double getZ() const { return z; }
    double getSpeed() const { return sqrt(vx*vx+vz*vz); }
    double getEnergy() const { return 0.5*shape.mass*(vx*vx+vz*vz)+0.5*shape.inertia*omega*omega; }
    double getRadius() const { return shape.radius; }
    double getMass() const { return shape.mass; }
    double getArea() const { return shape.area; }
    double getOmega() const { return omega; }
    double getVerticalSpeed() const { return vz; }
    int getSides() const { return (int)shape.local.size(); }
    double penetration(const HillSlope& hill) const {
        std::vector<Contact> c=contacts(hill,0); double depth=0;
        for(size_t i=0;i<c.size();++i) depth=max(depth,c[i].depth);
        return depth;
    }
};

// ------------------------------------------------------------------ 输入与回归验证（不新增源文件）
bool readShapeSettings(ShapeSettings& settings)
{
    for(;;) {
        TCHAR buf[160], defaults[160];
        formatText(defaults,160,_T("%d %d %.8g %.8g"),settings.kMin,settings.kMax,settings.r,settings.area);
        if(!InputBox(buf,160,_T("依次输入：最小边数 最大边数 圆半径r(m) 面积S(m^2)\n例如：3 6 2 3\n边数3~30；r为0.05~10m。S不得超过最小边数对应正多边形面积。\n每次R重新随机边数及顶点；G修改参数。厚度固定1m，初始无自旋。"),
            _T("随机凸多边形"),defaults,650,270,false)) return false;
        double values[4]; TCHAR* cursor=buf; bool valid=true;
        for(int i=0;i<4;++i) { TCHAR* end=cursor; errno=0; values[i]=_tcstod(cursor,&end); if(end==cursor || errno==ERANGE || !std::isfinite(values[i])) valid=false; cursor=end; }
        while(*cursor==_T(' ') || *cursor==_T('\t')) ++cursor;
        if(*cursor || !valid || values[0]!=floor(values[0]) || values[1]!=floor(values[1]) || values[0]<3 || values[0]>30 || values[1]<3 || values[1]>30) {
            MessageBox(GetHWnd(),_T("请输入四个有效数字，边数必须是3~30的整数。"),_T("输入错误"),MB_OK); continue;
        }
        ShapeSettings candidate={(int)values[0],(int)values[1],values[2],values[3]};
        try { validateShape(candidate); settings=candidate; return true; }
        catch(const std::exception& e) {
            TCHAR message[300]; formatText(message,300,_T("此参数组合不可用。\n给定最小边数允许的最大面积为 %.10g m^2。\nS需大于0，且S/r^2至少为0.0001；r为0.05~10m。"),maxPolygonArea(candidate.kMin,candidate.r));
            MessageBox(GetHWnd(),message,_T("参数超出范围"),MB_OK); (void)e;
        }
    }
}
int readTerrainSettings(TerrainSettings& settings)      //弹出输入框读地形参数：0 取消，1 自定义地形，2 恢复原始地形
{
    for (;;)
    {
        TCHAR buf[200], defaults[200];
        formatText(defaults, 200, _T("%.6g %.6g %.6g %.6g %.6g %.6g"),
            settings.topHeight, settings.steepDeg, settings.benchStart,
            settings.benchLength, settings.benchDeg, settings.toeLength);

        if (!InputBox(buf, 200, _T("依次输入：坡顶高度H(m) 陡坡角(度) 缓台起点(m) 缓台长度(m) 缓台坡角(度) 坡脚平台长度(m)\n例如：55.6 40 25 15 7 20\nH为5~60；陡坡角15~70；缓台坡角小于陡坡角；缓台长度填0表示没有缓台；总长不超过120m。\n只输入 0 恢复老师给的原始地形。"),
            _T("地形参数"), defaults, 700, 290, false))
        {
            return 0;
        }

        TCHAR* cursor = buf;

        while (*cursor == _T(' ') || *cursor == _T('\t'))
        {
            ++cursor;
        }

        if (cursor[0] == _T('0') && cursor[1] == 0)
        {
            return 2;
        }

        double values[6];
        bool valid = true;

        for (int i = 0; i < 6; ++i)
        {
            TCHAR* end = cursor;
            errno = 0;
            values[i] = _tcstod(cursor, &end);

            if (end == cursor || errno == ERANGE || !std::isfinite(values[i]))
            {
                valid = false;
            }

            cursor = end;
        }

        while (*cursor == _T(' ') || *cursor == _T('\t'))
        {
            ++cursor;
        }

        if (*cursor || !valid)
        {
            MessageBox(GetHWnd(), _T("请输入六个有效数字，用空格隔开。"), _T("输入错误"), MB_OK);
            continue;
        }

        TerrainSettings candidate;
        candidate.topHeight = values[0];
        candidate.steepDeg = values[1];
        candidate.benchStart = values[2];
        candidate.benchLength = values[3];
        candidate.benchDeg = values[4];
        candidate.toeLength = values[5];

        try
        {
            buildTerrain(candidate);
            settings = candidate;
            return 1;
        }
        catch (const std::exception& e)
        {
            MessageBoxA(GetHWnd(), e.what(), "Terrain parameters out of range", MB_OK | MB_ICONWARNING);
        }
    }
}

void requireTest(bool ok,const char* text) { if(!ok) throw std::runtime_error(text); }
int polygonSelfTest()
{
    try {
        std::mt19937 rng(20261111);
        double worstArea=0;
        for(int k=3;k<=12;++k) for(int trial=0;trial<30;++trial) {
            double r=0.2+randomUnit(rng)*2.8;
            double target=maxPolygonArea(k,r)*(0.35+0.5*randomUnit(rng));
            PolygonShape p=generatePolygon(k,r,target,rng);
            worstArea=max(worstArea,fabs(p.area-target)/(r*r));
            requireTest(p.local.size()==(size_t)k,"side count");
            requireTest(fabs(p.area-target)<1e-9*r*r,"area");
            for(int i=0;i<k;++i) {
                Point q=minus2(p.local[i],p.circleCenter);
                requireTest(fabs(sqrt(dot2(q,q))-r)<1e-9*r,"circle radius");
                Point a=p.local[i],b=p.local[(i+1)%k],c=p.local[(i+2)%k];
                requireTest(cross2(minus2(b,a),minus2(c,b))>0,"strict convexity");
            }
        }
        // 用户复现：10 20 7 20，每个可选边数都必须能生成，不能偷偷换边数。
        for(int k=10;k<=20;++k) for(int trial=0;trial<100;++trial) {
            PolygonShape p=generatePolygon(k,7,20,rng);
            requireTest(p.local.size()==(size_t)k,"10 20 7 20 side count");
            requireTest(fabs(p.area-20)<1e-8,"10 20 7 20 area");
            for(int i=0;i<k;++i) {
                Point q=minus2(p.local[i],p.circleCenter);
                requireTest(fabs(sqrt(dot2(q,q))-7)<1e-9,"10 20 7 20 radius");
                Point a=p.local[i],b=p.local[(i+1)%k],c=p.local[(i+2)%k];
                requireTest(cross2(minus2(b,a),minus2(c,b))>0,"10 20 7 20 convexity");
            }
        }
        PolygonShape square=generatePolygon(4,1,2,rng);
        requireTest(fabs(square.inertia/square.mass-1.0/3)<1e-10,"square inertia");
        bool rejected=false;
        try { validateShape(ShapeSettings{3,6,1,2}); } catch(...) { rejected=true; }
        requireTest(rejected,"impossible area rejected");
        DynamicArray<Point> flat; flat.push_back(Point(-100,0)); flat.push_back(Point(100,0)); HillSlope floorHill(flat);
        RollingStone drop(0,5,square); double maxPen=0;
        for(int i=0;i<10000;++i) { drop.update(floorHill,DT); maxPen=max(maxPen,drop.penetration(floorHill)); }
        requireTest(maxPen<0.005,"floor penetration"); requireTest(!drop.isMoving(),"body settles on floor");
        double oldDrag=g_dragCoef,oldMu=g_slideFriction,oldRest=g_restitution,oldRoll=g_rollResist;
        g_dragCoef=0; g_slideFriction=0; g_restitution=0.6; g_rollResist=0;
        std::vector<Point> corners; double h=sqrt(0.5);
        corners.push_back(Point(-h,-h)); corners.push_back(Point(h,-h));
        corners.push_back(Point(h,h)); corners.push_back(Point(-h,h));
        PolygonShape alignedSquare(corners,1);
        RollingStone bounce(0,5,alignedSquare); bool bounced=false;
        for(int i=0;i<3000;++i) {
            double before=bounce.getVerticalSpeed(); bounce.update(floorHill,DT);
            if(before< -1 && bounce.getVerticalSpeed()>0) {
                requireTest(fabs(bounce.getVerticalSpeed()/(-before)-0.6)<0.01,"restitution response");
                requireTest(fabs(bounce.getOmega())<0.01,"symmetric contact torque"); bounced=true; break;
            }
        }
        requireTest(bounced,"drop bounces");
        RollingStone falling(0,100,alignedSquare);
        for(int i=0;i<100;++i) falling.update(floorHill,DT);
        requireTest(fabs(falling.getVerticalSpeed()+GRAVITY*DT*100)<1e-9,"free-fall acceleration");
        requireTest(fabs(falling.getOmega())<1e-12,"no torque in free fall");
        DynamicArray<Point> ridge; ridge.push_back(Point(-10,0)); ridge.push_back(Point(0,1)); ridge.push_back(Point(10,0));
        HillSlope ridgeHill(ridge); RollingStone ridgeBody(0,0,alignedSquare); ridgeBody.adjustToGround(ridgeHill);
        requireTest(ridgeBody.penetration(ridgeHill)<0.0001,"terrain vertex under polygon edge");
        g_dragCoef=oldDrag; g_slideFriction=oldMu; g_restitution=oldRest; g_rollResist=oldRoll;
        for(int k=3;k<=30;++k) {
            PolygonShape regular=generatePolygon(k,2,maxPolygonArea(k,2),rng);
            requireTest(fabs(regular.area-maxPolygonArea(k,2))<1e-9,"maximum-area regular polygon");
        }
        std::mt19937 same1(123),same2(123);
        PolygonShape repeat1=generatePolygon(ShapeSettings{3,8,2,3},same1),repeat2=generatePolygon(ShapeSettings{3,8,2,3},same2);
        requireTest(repeat1.local.size()==repeat2.local.size(),"deterministic side count");
        for(size_t i=0;i<repeat1.local.size();++i) requireTest(dot2(minus2(repeat1.local[i],repeat2.local[i]),minus2(repeat1.local[i],repeat2.local[i]))<1e-20,"repeatable sample");
        HillSlope hill;
        for(int j=0;j<12;++j) {
            PolygonShape p=generatePolygon(ShapeSettings{3,8,2,3},rng);
            RollingStone body(2,hill.getHeight(2),p); body.adjustToGround(hill); body.startAlongSlope(hill,1.5);
            for(int i=0;i<20000 && body.isMoving() && body.getX()<hill.getEndX();++i) {
                body.update(hill,DT); maxPen=max(maxPen,body.penetration(hill));
                requireTest(body.getEnergy()<p.mass*GRAVITY*100,"no explosive energy gain");
            }
        }
        // 小面积/多顶点的运动也走真实接触求解器，不只验证外形。
        for(int k=10;k<=20;++k) {
            PolygonShape p=generatePolygon(k,7,20,rng);
            RollingStone body(2,hill.getHeight(2),p); body.adjustToGround(hill); body.startAlongSlope(hill,1.5);
            for(int i=0;i<20000 && body.isMoving() && body.getX()<hill.getEndX();++i) {
                body.update(hill,DT); maxPen=max(maxPen,body.penetration(hill));
                requireTest(body.getEnergy()<p.mass*GRAVITY*100,"10 20 7 20 stable motion");
            }
        }
        // 边数上下限、低面积、接近最大面积，防止在另一端出现新的拒绝采样失败。
        for(int k=3;k<=30;++k) for(int mode=0;mode<3;++mode) {
            double r=7;
            double target=mode==0?1.0001e-4*r*r:maxPolygonArea(k,r)*(mode==1?0.99:0.999999);
            PolygonShape p=generatePolygon(k,r,target,rng);
            requireTest(fabs(p.area-target)<1e-8,"extreme area accuracy");
            for(int i=0;i<k;++i) {
                Point a=p.local[i],b=p.local[(i+1)%k],c=p.local[(i+2)%k];
                requireTest(cross2(minus2(b,a),minus2(c,b))>0,"extreme area convexity");
            }
        }
        TerrainSettings terrainDefault;
        DynamicArray<Point> terrainPoints = buildTerrain(terrainDefault);
        requireTest(fabs(terrainPoints[0].z - terrainDefault.topHeight) < 1e-12, "terrain top height");
        requireTest(fabs(terrainPoints[terrainPoints.size() - 1].z) < 1e-12, "terrain toe height");
        requireTest(terrainPoints[terrainPoints.size() - 1].x <= TERRAIN_MAX_LENGTH, "terrain length");

        for (size_t i = 1; i < terrainPoints.size(); ++i)
        {
            requireTest(terrainPoints[i].x > terrainPoints[i - 1].x, "terrain x increases");
            requireTest(terrainPoints[i].z <= terrainPoints[i - 1].z, "terrain goes downhill");
        }

        TerrainSettings terrainNoBench;
        terrainNoBench.benchLength = 0;
        requireTest(buildTerrain(terrainNoBench).size() == 4, "terrain without bench");

        TerrainSettings terrainBad;
        terrainBad.steepDeg = 80;
        bool terrainRejected = false;

        try
        {
            buildTerrain(terrainBad);
        }
        catch (...)
        {
            terrainRejected = true;
        }

        requireTest(terrainRejected, "invalid terrain rejected");

        HillSlope customHill(terrainPoints);

        for (int j = 0; j < 6; ++j)
        {
            PolygonShape p = generatePolygon(ShapeSettings{3, 8, 2, 3}, rng);
            RollingStone body(customHill.getPeakX(), customHill.getHeight(customHill.getPeakX()), p);
            body.adjustToGround(customHill);
            body.startAlongSlope(customHill, 1.5);

            for (int i = 0; i < 20000 && body.isMoving() && body.getX() < customHill.getEndX(); ++i)
            {
                body.update(customHill, DT);
                maxPen = max(maxPen, body.penetration(customHill));
            }
        }

        requireTest(maxPen<0.01,"slope penetration");
        std::printf("PASS: 300 random geometries + 1100 samples for input 10 20 7 20, area/radius/convexity, inertia, invalid input, restitution, free fall, ridge contact, regular limit, deterministic seed, drop/rest, 23 slope trajectories and 84 extreme-area samples. Max area/r^2 error=%.3g; max penetration=%.6g m\n",worstArea,maxPen);
        return 0;
    } catch(const std::exception& e) { std::fprintf(stderr,"FAIL: %s\n",e.what()); return 1; }
}

// ------------------------------------------------------------------ 主程序
int main(int argc, char** argv)
{
    if(argc>1 && std::strcmp(argv[1],"--self-test")==0) return polygonSelfTest();
	initgraph(WIDTH, HEIGHT);
	setbkcolor(RGB(135, 206, 235));
    SetWindowText(GetHWnd(),_T("Rolling Stone - Convex Polygon"));
	BeginBatchDraw();

	HillSlope hill;
	int initX = hill.getPeakX();
	double initZ = hill.getHeight((double)initX);

    ShapeSettings shapeSettings={3,6,2.0,3.0};
    std::mt19937 shapeRng((unsigned int)GetTickCount() ^ (unsigned int)GetCurrentProcessId());
    if(!readShapeSettings(shapeSettings)) { EndBatchDraw(); closegraph(); return 0; }
    double stoneSpeed=1.5;
    TerrainSettings terrainSettings;      //当前地形参数（按 T 修改）

    if (readTerrainSettings(terrainSettings) == 1)
    {
        hill = HillSlope(buildTerrain(terrainSettings));
        initX = hill.getPeakX();
        initZ = hill.getHeight((double)initX);
    }

    PolygonShape initial=generatePolygon(3,2,3,shapeRng);
    for(;;) {
        try { initial=generatePolygon(shapeSettings,shapeRng); break; }
        catch(const std::exception& e) {
            MessageBoxA(GetHWnd(),e.what(),"Shape generation failed",MB_OK|MB_ICONWARNING);
            if(!readShapeSettings(shapeSettings)) { EndBatchDraw(); closegraph(); return 0; }
        }
    }
    RollingStone stone((double)initX,initZ,initial);
    stone.adjustToGround(hill); stone.startAlongSlope(hill,stoneSpeed);
	bool simEnded = false;

	bool paused = false;
	double stoppedDuration = 0.0;
	bool stoppedTriggered = false;
	bool inMenu = false;
	int menuSel = 0; // 0: c_rr 1: μ 2: e 3: Cd 4: 倍速
	const int menuRows = 4;
	const int playbackOptions[4] = {1, 2, 4, 8};
	int playbackIndex = 0;
	bool inPauseMenu = false;
	int pauseMenuSel = 0;

	char inputBuf[4][32];
	std::snprintf(inputBuf[0], sizeof(inputBuf[0]), "%.3f", g_rollResist);
	std::snprintf(inputBuf[1], sizeof(inputBuf[1]), "%.3f", g_slideFriction);
	std::snprintf(inputBuf[2], sizeof(inputBuf[2]), "%.3f", g_restitution);
	std::snprintf(inputBuf[3], sizeof(inputBuf[3]), "%.3f", g_dragCoef);

	DynamicArray<TimeSpeed> speedData;
	double simTime = 0.0;
	const double graphWindow = 10.0;

	settextstyle(16, 0, _T("Consolas"));
	setcolor(RGB(50, 50, 50));
	outtextxy(10, 20, _T("Convex polygon rigid body - SI units; thickness=1 m"));
	outtextxy(10, 45, _T("SPACE=Pause M=Physics R=New polygon G=Geometry T=Terrain ESC=Exit"));

	while (true)
	{
        int key=-1;
        ExMessage event;
        if(peekmessage(&event,EX_CHAR)) key=(int)event.ch;
        else if(_kbhit()) key=_getch();
        if(key>=0)
        {
			if (key == 27)
				break;
			if (key == 32)
			{
				paused = !paused;
			}
			if (key == 'p' || key == 'P')
			{
				inPauseMenu = !inPauseMenu;
				pauseMenuSel = paused ? 1 : 0;
			}
			if (inPauseMenu)
			{
				if (key == 'w' || key == 'W' || key == 's' || key == 'S')
				{
					pauseMenuSel = 1 - pauseMenuSel;
				}
				if (key == 13)
				{
					if (pauseMenuSel == 0)
						paused = true;
					else
						paused = false;
					inPauseMenu = false;
				}
			}
            if(key=='r' || key=='R' || key=='g' || key=='G')
            {
                ShapeSettings candidate=shapeSettings;
                bool apply=(key=='r' || key=='R') || readShapeSettings(candidate);
                if(apply) try {
                    PolygonShape generated=generatePolygon(candidate,shapeRng);
                    stone=RollingStone((double)initX,initZ,generated);
                    stone.adjustToGround(hill); stone.startAlongSlope(hill,stoneSpeed);
                    shapeSettings=candidate;
                    paused=false; simEnded=false; inMenu=false; inPauseMenu=false;
                    stoppedDuration=0; stoppedTriggered=false; speedData.clear(); simTime=0;
                } catch(const std::exception& e) {
                    MessageBoxA(GetHWnd(),e.what(),"Shape generation failed - previous shape retained",MB_OK|MB_ICONWARNING);
                }
            }
            if (key == 't' || key == 'T')
            {
                int choice = readTerrainSettings(terrainSettings);

                if (choice != 0)
                {
                    try
                    {
                        HillSlope newHill = (choice == 2) ? HillSlope() : HillSlope(buildTerrain(terrainSettings));
                        hill = newHill;
                        initX = hill.getPeakX();
                        initZ = hill.getHeight((double)initX);

                        stone = RollingStone((double)initX, initZ, generatePolygon(shapeSettings, shapeRng));
                        stone.adjustToGround(hill);
                        stone.startAlongSlope(hill, stoneSpeed);

                        paused = false; simEnded = false; inMenu = false; inPauseMenu = false;
                        stoppedDuration = 0; stoppedTriggered = false; speedData.clear(); simTime = 0;
                    }
                    catch (const std::exception& e)
                    {
                        MessageBoxA(GetHWnd(), e.what(), "Terrain change failed - previous terrain retained", MB_OK | MB_ICONWARNING);
                    }
                }
            }
			if (key == 'm' || key == 'M')
			{
				inMenu = !inMenu;
				playbackIndex = 0;
				for (int i = 0; i < 4; i++)
					if (playbackOptions[i] == g_playbackFactor)
						playbackIndex = i;
			}
			if (inMenu)
			{
				if (key == 'w' || key == 'W')
				{
					menuSel = (menuSel + menuRows) % (menuRows + 1);
				}
				if (key == 's' || key == 'S')
				{
					menuSel = (menuSel + 1) % (menuRows + 1);
				}
				if (key == 'a' || key == 'A')
				{
					if (menuSel == 0)
						g_rollResist = max(0.0, g_rollResist - 0.01);
					if (menuSel == 1)
						g_slideFriction = max(0.0, g_slideFriction - 0.05);
					if (menuSel == 2)
						g_restitution = max(0.0, g_restitution - 0.05);
					if (menuSel == 3)
						g_dragCoef = max(0.0, g_dragCoef - 0.1);
					if (menuSel == 4)
					{
						playbackIndex = max(0, playbackIndex - 1);
						g_playbackFactor = playbackOptions[playbackIndex];
					}
				}
				if (key == 'd' || key == 'D')
				{
					if (menuSel == 0)
						g_rollResist = min(0.30, g_rollResist + 0.01);
					if (menuSel == 1)
						g_slideFriction = min(1.00, g_slideFriction + 0.05);
					if (menuSel == 2)
						g_restitution = min(0.90, g_restitution + 0.05);
					if (menuSel == 3)
						g_dragCoef = min(2.00, g_dragCoef + 0.1);
					if (menuSel == 4)
					{
						playbackIndex = min(3, playbackIndex + 1);
						g_playbackFactor = playbackOptions[playbackIndex];
					}
				}
                if(key=='a'||key=='A'||key=='d'||key=='D') {
                    std::snprintf(inputBuf[0],32,"%.3f",g_rollResist);
                    std::snprintf(inputBuf[1],32,"%.3f",g_slideFriction);
                    std::snprintf(inputBuf[2],32,"%.3f",g_restitution);
                    std::snprintf(inputBuf[3],32,"%.3f",g_dragCoef);
                }
				if (key == 13)
				{
					inMenu = false;
				}
			}
		}

		{
            if(!paused && !inMenu && !inPauseMenu && !simEnded)
            {
			// 每帧 8 个子步（2 ms × 8 = 16 ms），倍速就是“每帧多跑几步”
			int steps = STEPS_PER_FRAME * g_playbackFactor;
			for (int i = 0; i < steps; ++i)
			{
                try { stone.update(hill, DT); }
                catch(const std::exception& e) {
                    paused=true; simEnded=true;
                    MessageBoxA(GetHWnd(),e.what(),"Simulation stopped",MB_OK|MB_ICONWARNING); break;
                }
				simTime += DT;
			}
			double curSpeed = stone.getSpeed();
			speedData.push_back(TimeSpeed(simTime, curSpeed));
			while (!speedData.empty() && speedData[0].t < simTime - graphWindow)
			{
				speedData.erase_front();
			}

			if (!simEnded)
			{
				if (stone.getX() >= hill.getEndX() - 0.5)
				{
                    simEnded=true; paused=true;
				}
			}

			if (!stoppedTriggered && !simEnded)
			{
				if (!stone.isMoving())
					stoppedDuration += STEPS_PER_FRAME * DT * g_playbackFactor;
				else
					stoppedDuration = 0.0;
				if (stoppedDuration >= 3.0)
				{
                    simEnded=true; paused=true;
				}
			}

            }
			cleardevice();
			setbkcolor(RGB(135, 206, 235));

			hill.draw();
			if (inMenu)
			{
				setfillcolor(RGB(240, 240, 240));
				bar(70, 45, 500, 440);
				setcolor(RGB(0, 0, 0));
				settextstyle(16, 0, _T("Consolas"));
				outtextxy(90, 55, _T("Parameters (W/S select, A/D change, Enter apply)"));

				TCHAR bufMenu[160];
				for (int i = 0; i < menuRows; i++)
				{
					int y = 100 + i * 55;
					if (i == menuSel)
						setcolor(RGB(255, 0, 0));
					else
						setcolor(RGB(0, 0, 0));
					if (i == 0)
						formatText(bufMenu, 160, _T("滚动阻矩系数 (0-0.30):"));
					else if (i == 1)
						formatText(bufMenu, 160, _T("摩擦系数 mu (0-1):"));
					else if (i == 2)
						formatText(bufMenu, 160, _T("恢复系数 e (0-0.90):"));
					else
						formatText(bufMenu, 160, _T("空气阻力 Cd (0=真空):"));
					outtextxy(90, y, bufMenu);

					setcolor(RGB(0, 0, 0));
					TCHAR tbuf[64];
					#ifdef _UNICODE
                    MultiByteToWideChar(CP_ACP, 0, inputBuf[i], -1, tbuf, 64);
#else
                    std::snprintf(tbuf, sizeof(tbuf), "%s", inputBuf[i]);
#endif
					outtextxy(400, y, tbuf);
				}
				int y = 100 + menuRows * 55;
				if (menuSel == menuRows)
					setcolor(RGB(255, 0, 0));
				else
					setcolor(RGB(0, 0, 0));
				formatText(bufMenu, 160, _T("播放倍速: x1 x2 x4 x8 (当前 x%d)"), g_playbackFactor);
				outtextxy(90, y, bufMenu);

				setfillcolor(RGB(100, 200, 100));
				bar(250, 380, 380, 415);
				setcolor(RGB(0, 0, 0));
				outtextxy(275, 388, _T("启动模拟"));


				setcolor(RGB(80, 80, 80));
				outtextxy(90, 360, _T("按 R 重置模拟，按 M 退出菜单"));
			}

			if (inPauseMenu)
			{
				setfillcolor(RGB(240, 240, 240));
				bar(WIDTH / 2 - 120, HEIGHT / 2 - 40, WIDTH / 2 + 120, HEIGHT / 2 + 40);
				setcolor(RGB(0, 0, 0));
				settextstyle(16, 0, _T("Consolas"));
				outtextxy(WIDTH / 2 - 60, HEIGHT / 2 - 24, _T("Pause Menu (W/S select, Enter)"));
				if (pauseMenuSel == 0)
					setcolor(RGB(255, 0, 0));
				else
					setcolor(RGB(0, 0, 0));
				outtextxy(WIDTH / 2 - 40, HEIGHT / 2 - 2, _T("Pause"));
				if (pauseMenuSel == 1)
					setcolor(RGB(255, 0, 0));
				else
					setcolor(RGB(0, 0, 0));
				outtextxy(WIDTH / 2 - 40, HEIGHT / 2 + 18, _T("Continue"));
			}

            stone.draw();
            TCHAR geometryInfo[200];
            formatText(geometryInfo,200,_T("k=%d  r=%.4g m  S=%.8g m2  omega=%.3f rad/s"),stone.getSides(),stone.getRadius(),stone.getArea(),stone.getOmega());
            settextstyle(16,0,_T("Consolas")); setcolor(RGB(20,30,50));
            outtextxy(10,10,geometryInfo);
            outtextxy(10,32,_T("SPACE:pause  R:new polygon  G:geometry  T:terrain  M:physics  ESC:exit"));

			// -------- HUD（米制） --------
			TCHAR info[200];
			setcolor(RGB(255, 255, 255));
			settextstyle(14, 0, _T("Consolas"));

			formatText(info, 200, _T("Position: %.1f m"), stone.getX());
			outtextxy(10, HEIGHT - 100, info);

			formatText(info, 200, _T("Height: %.1f m"), stone.getZ());
			outtextxy(10, HEIGHT - 80, info);

			formatText(info, 200, _T("Speed: %.1f m/s"), stone.getSpeed());
			outtextxy(10, HEIGHT - 60, info);

			formatText(info, 200, _T("Energy: %.1f kJ"), stone.getEnergy() / 1000.0);
			outtextxy(10, HEIGHT - 40, info);

			formatText(info, 200, _T("Circle r: %.2f m   Mass: %.0f kg"), stone.getRadius(), stone.getMass());
			outtextxy(10, HEIGHT - 20, info);

			formatText(info, 200, _T("Status: %s"), stone.isMoving() ? _T("ROLLING") : _T("STOPPED"));
			setcolor(stone.isMoving() ? RGB(255, 200, 0) : RGB(100, 255, 100));
			outtextxy(10, HEIGHT - 120, info);

			double slope = hill.getSlopeAngle(stone.getX()) * 180 / 3.14159;
			setcolor(RGB(200, 200, 200));
			formatText(info, 200, _T("Slope: %.1f deg"), slope);
			outtextxy(WIDTH - 130, HEIGHT - 60, info);

			TCHAR paramBuf[240];
			formatText(paramBuf, 240, _T("c_rr:%.2f mu:%.2f e:%.2f Cd:%.1f x%d"),
						g_rollResist, g_slideFriction, g_restitution, g_dragCoef, g_playbackFactor);
			setcolor(RGB(180, 180, 180));
			settextstyle(12, 0, _T("Consolas"));
			outtextxy(WIDTH - 330, 10, paramBuf);

			// 速度条（0~20 m/s）
			int speedBar = (int)(stone.getSpeed() / 20.0 * 100);
			if (speedBar > 100)
				speedBar = 100;
			setfillcolor(RGB(255, 100, 100));
			bar(WIDTH - 120, HEIGHT - 40, WIDTH - 120 + speedBar, HEIGHT - 30);
			setcolor(RGB(150, 150, 150));
			rectangle(WIDTH - 120, HEIGHT - 40, WIDTH - 20, HEIGHT - 30);

			// -------- 速度-时间曲线 --------
			const int gx = WIDTH - 260;
			const int gy = 20;
			const int gw = 240;
			const int gh = 120;

			setfillcolor(RGB(135, 206, 235));
			bar(gx, gy, gx + gw, gy + gh);
			setcolor(RGB(200, 200, 200));
			rectangle(gx, gy, gx + gw, gy + gh);

			const int marginL = 36;
			const int marginR = 8;
			const int marginTop = 6;
			const int marginBottom = 20;
			int plotX = gx + marginL;
			int plotY = gy + marginTop;
			int plotW = gw - marginL - marginR;
			int plotH = gh - marginTop - marginBottom;

			double t0 = simTime - graphWindow;
			if (t0 < 0)
				t0 = 0;

			setcolor(RGB(180, 180, 180));
			line(plotX, plotY, plotX, plotY + plotH);
			line(plotX, plotY + plotH, plotX + plotW, plotY + plotH);

			double gridStart = floor(t0);
			double gridEnd = ceil(simTime);
			setcolor(RGB(220, 220, 220));
			setlinestyle(PS_DASH, 1);
			for (int g = (int)gridStart; g <= (int)gridEnd; ++g)
			{
				double gxTime = (double)g;
				double gxp = plotX + (gxTime - (simTime - graphWindow)) / graphWindow * plotW;
				if (gxp >= plotX && gxp <= plotX + plotW)
				{
					line((int)gxp, plotY, (int)gxp, plotY + plotH);
				}
			}
			setlinestyle(PS_SOLID, 1);

			double yMax = 8.0; // 纵轴最小量程 8 m/s
			for (size_t _i = 0; _i < speedData.size(); ++_i)
			{
				auto &p = speedData[_i];
				if (p.s > yMax)
					yMax = p.s;
			}
			yMax *= 1.1;

			settextstyle(12, 0, _T("Consolas"));
			setcolor(RGB(80, 80, 80));
			TCHAR buf[64];
			formatText(buf, 64, _T("%.1f"), yMax);
			outtextxy(plotX - 32, plotY - 8, buf);
			formatText(buf, 64, _T("%.1f"), 0.0);
			outtextxy(plotX - 28, plotY + plotH - 14, buf);

			formatText(buf, 64, _T("%.1fs"), t0);
			outtextxy(plotX + 2, plotY + plotH + 2, buf);
			formatText(buf, 64, _T("%.1fs"), simTime);
			outtextxy(plotX + plotW - 40, plotY + plotH + 2, buf);

			formatText(buf, 64, _T("Speed (m/s)"));
			outtextxy(plotX + 2, plotY - 18, buf);
			formatText(buf, 64, _T("Time (s)"));
			outtextxy(plotX + plotW / 2 - 22, plotY + plotH + 2, buf);

			if (speedData.size() >= 2)
			{
				setcolor(RGB(255, 0, 0));
				setlinestyle(PS_SOLID, 2);
				for (size_t i = 1; i < speedData.size(); ++i)
				{
					double tx1 = speedData[i - 1].t;
					double ty1 = speedData[i - 1].s;
					double tx2 = speedData[i].t;
					double ty2 = speedData[i].s;

					double nx1 = plotX + (tx1 - (simTime - graphWindow)) / graphWindow * plotW;
					double nx2 = plotX + (tx2 - (simTime - graphWindow)) / graphWindow * plotW;
					double ny1 = plotY + plotH - (ty1 / yMax) * plotH;
					double ny2 = plotY + plotH - (ty2 / yMax) * plotH;

					if (nx1 < plotX)
						nx1 = plotX;
					if (nx1 > plotX + plotW)
						nx1 = plotX + plotW;
					if (nx2 < plotX)
						nx2 = plotX;
					if (nx2 > plotX + plotW)
						nx2 = plotX + plotW;
					if (ny1 < plotY)
						ny1 = plotY;
					if (ny1 > plotY + plotH)
						ny1 = plotY + plotH;
					if (ny2 < plotY)
						ny2 = plotY;
					if (ny2 > plotY + plotH)
						ny2 = plotY + plotH;

					line((int)nx1, (int)ny1, (int)nx2, (int)ny2);
				}
				setlinestyle(PS_SOLID, 1);
			}

			FlushBatchDraw();
			Sleep(16);
		}

		if (!stone.isMoving() || simEnded)
		{
			setcolor(RGB(0, 255, 0));
			settextstyle(16, 0, _T("Consolas"));
			outtextxy(WIDTH / 2 - 80, HEIGHT - 150, _T("Ended: R=new polygon, G=geometry, T=terrain"));
		}
	}

	closegraph();
	return 0;
}
