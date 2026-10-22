#pragma once
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mpt {
constexpr float pi = 3.14159265358979323846f;

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
    Vec3 operator+(Vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(Vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return *this * (1.0f / s); }
    Vec3 operator*(Vec3 b) const { return {x*b.x,y*b.y,z*b.z}; }
    Vec3& operator+=(Vec3 b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    Vec3& operator*=(Vec3 b) { x*=b.x; y*=b.y; z*=b.z; return *this; }
};
inline Vec3 operator*(float s, Vec3 v) { return v*s; }
inline float dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline float length2(Vec3 v) { return dot(v,v); }
float length(Vec3 v);
Vec3 normalize(Vec3 v);
inline Vec3 clamp(Vec3 a, float lo, float hi) { return {std::clamp(a.x,lo,hi),std::clamp(a.y,lo,hi),std::clamp(a.z,lo,hi)}; }

struct Ray { Vec3 origin, direction; };
struct Material { Vec3 albedo{0.8f,0.8f,0.8f}; Vec3 emission{0,0,0}; };
struct Triangle {
    Vec3 a,b,c;
    Vec3 normal;
    int material = 0;
    Triangle(Vec3 A, Vec3 B, Vec3 C, int m);
};
struct Hit { float t = 1e30f; Vec3 position, normal; int material=-1; };
struct AABB { Vec3 lo{1e30f,1e30f,1e30f}; Vec3 hi{-1e30f,-1e30f,-1e30f}; };
struct BvhNode { AABB bounds; int left=-1, right=-1, start=0, count=0; };

class Scene {
public:
    std::vector<Material> materials;
    std::vector<Triangle> triangles;
    // Rectangular, single-sided ceiling light (in the XZ plane, points down).
    Vec3 light_corner, light_edge_u, light_edge_v, light_emission;
    float light_area = 0;

    void build_cornell_box();
    // Move the rectangular emitter (including visible geometry), then rebuild its BVH.
    void translate_light(Vec3 offset);
    bool intersect(const Ray& ray, float near_t, float far_t, Hit& result) const;
    bool occluded(const Ray& ray, float near_t, float far_t) const;
    void build_bvh();
    size_t bvh_node_count() const { return nodes_.size(); }
private:
    size_t light_first_triangle_ = 0;
    std::vector<BvhNode> nodes_;
    std::vector<int> order_;
    int build_node(int start,int end);
    void quad(Vec3 a,Vec3 b,Vec3 c,Vec3 d,int material);
    void box(Vec3 center,Vec3 half,float y_rotation,int material);
};

struct Settings {
    int width=480, height=480, samples=128, max_bounces=8, threads=0;
    float fov_degrees=40.0f, exposure=1.0f, light_scale=1.0f;
    Vec3 camera_position{0,1.0f,3.35f}, camera_target{0,1.0f,0}, camera_up{0,1,0};
    Vec3 light_offset{0,0,0}; // emitter XYZ relative to initial Cornell Box position
    uint64_t seed=42;
};
class Image {
public:
    int width=0,height=0;
    std::vector<Vec3> linear;
    Image(int w,int h);
    std::vector<uint8_t> rgb8(float exposure) const;
    bool save_ppm(const std::string& filename, float exposure) const;
    bool save_bmp(const std::string& filename, float exposure) const;
    bool save(const std::string& filename, float exposure) const;
};

class Renderer {
public:
    explicit Renderer(Scene scene);
    // Callback receives a finished progressive pass and current sample count.
    // Called on the render thread and should return quickly.
    void render(const Settings& settings, Image& result,
                const std::function<void(int,const Image&)>& on_pass = {},
                const std::atomic<bool>* stop = nullptr) const;
private:
    Scene scene_;
};
}