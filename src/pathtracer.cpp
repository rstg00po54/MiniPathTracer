#include "pathtracer.h"
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <thread>

namespace mpt {
float length(Vec3 v) { return std::sqrt(length2(v)); }
Vec3 normalize(Vec3 v) { const float n=length(v); return n>0? v/n : Vec3{}; }
Triangle::Triangle(Vec3 A,Vec3 B,Vec3 C,int m):a(A),b(B),c(C),normal(normalize(cross(b-a,c-a))),material(m){}

namespace {
constexpr float eps=0.0001f;
struct Rng {
    uint64_t state;
    explicit Rng(uint64_t seed):state(seed){}
    uint32_t u32() {
        state+=0x9e3779b97f4a7c15ull;
        uint64_t z=state;
        z=(z^(z>>30))*0xbf58476d1ce4e5b9ull;
        z=(z^(z>>27))*0x94d049bb133111ebull;
        return static_cast<uint32_t>((z^(z>>31))>>32);
    }
    float unit() { return (u32()>>8)*(1.0f/16777216.0f); }
};
AABB triangle_bounds(const Triangle& t) {
    AABB b;
    for(Vec3 p : {t.a,t.b,t.c}) {
        b.lo={std::min(b.lo.x,p.x),std::min(b.lo.y,p.y),std::min(b.lo.z,p.z)};
        b.hi={std::max(b.hi.x,p.x),std::max(b.hi.y,p.y),std::max(b.hi.z,p.z)};
    }
    return b;
}
AABB merge(AABB a,AABB b) {
    a.lo={std::min(a.lo.x,b.lo.x),std::min(a.lo.y,b.lo.y),std::min(a.lo.z,b.lo.z)};
    a.hi={std::max(a.hi.x,b.hi.x),std::max(a.hi.y,b.hi.y),std::max(a.hi.z,b.hi.z)};
    return a;
}
float axis(Vec3 v,int a){return a==0?v.x:(a==1?v.y:v.z);}
bool box_hit(const AABB& b,const Ray& r,float tmin,float tmax) {
    for(int a=0;a<3;++a) {
        const float o=axis(r.origin,a), d=axis(r.direction,a);
        const float lo=axis(b.lo,a), hi=axis(b.hi,a);
        if(std::abs(d)<1e-12f) { if(o<lo||o>hi)return false; continue; }
        float t0=(lo-o)/d, t1=(hi-o)/d;
        if(t0>t1)std::swap(t0,t1);
        tmin=std::max(tmin,t0); tmax=std::min(tmax,t1);
        if(tmax<tmin)return false;
    }
    return true;
}
bool triangle_hit(const Triangle& tri,const Ray& ray,float tmin,float tmax,float& dist){
    Vec3 e1=tri.b-tri.a,e2=tri.c-tri.a;
    Vec3 p=cross(ray.direction,e2);
    float det=dot(e1,p);
    if(std::abs(det)<1e-8f)return false;
    float inv=1.0f/det;
    Vec3 t=ray.origin-tri.a;
    float u=dot(t,p)*inv;
    if(u<0||u>1)return false;
    float v=dot(ray.direction,cross(t,e1))*inv;
    if(v<0||u+v>1)return false;
    float distance=dot(e2,cross(t,e1))*inv;
    if(distance<tmin||distance>tmax)return false;
    dist=distance;return true;
}
Vec3 cosine_hemisphere(Vec3 n,Rng& rng){
    float u=rng.unit(),v=rng.unit();
    float r=std::sqrt(u),angle=2*pi*v;
    Vec3 t=normalize(cross(std::abs(n.y)<0.9f?Vec3{0,1,0}:Vec3{1,0,0},n));
    Vec3 b=cross(n,t);
    return normalize(t*(r*std::cos(angle))+b*(r*std::sin(angle))+n*std::sqrt(1-u));
}
float max_component(Vec3 c){return std::max({c.x,c.y,c.z});}
Vec3 direct_light(const Scene& s,const Hit& hit,const Material& mat,Rng& rng,float light_scale) {
    Vec3 light_pos=s.light_corner+s.light_edge_u*rng.unit()+s.light_edge_v*rng.unit();
    Vec3 delta=light_pos-hit.position;
    float distance2=length2(delta);
    if(distance2<eps*eps)return {};
    float distance=std::sqrt(distance2);
    Vec3 wi=delta/distance;
    const Vec3 light_normal{0,-1,0};
    const float cos_surface=std::max(0.0f,dot(hit.normal,wi));
    const float cos_light=std::max(0.0f,dot(light_normal,-wi));
    if(cos_surface<=0||cos_light<=0)return {};
    Ray shadow{hit.position+hit.normal*eps,wi};
    if(s.occluded(shadow,eps,distance-2*eps))return {};
    return mat.albedo * s.light_emission * (light_scale*(cos_surface*cos_light*s.light_area)/(pi*distance2));
}
Vec3 trace(const Scene& s,Ray ray,Rng& rng,int bounces,float light_scale){
    Vec3 radiance{},throughput{1,1,1};
    for(int bounce=0;bounce<bounces;++bounce){
        Hit hit;
        if(!s.intersect(ray,eps,1e30f,hit))break;
        const Material& m=s.materials[static_cast<size_t>(hit.material)];
        // All indirect paths use next-event estimation; adding emission at every
        // diffuse hit would double-count the area light sampled below.
        if(max_component(m.emission)>0) {
            if(bounce==0) radiance+=throughput*m.emission*light_scale;
            break;
        }
        radiance+=throughput*direct_light(s,hit,m,rng,light_scale);
        throughput=throughput*m.albedo;
        if(bounce>=3){
            const float survive=std::clamp(max_component(throughput),0.1f,0.95f);
            if(rng.unit()>survive)break;
            throughput=throughput/survive;
        }
        ray={hit.position+hit.normal*eps,cosine_hemisphere(hit.normal,rng)};
    }
    return radiance;
}
Vec3 tone(Vec3 c,float exposure){
    c=clamp(c*exposure,0.0f,1e10f);
    auto op=[](float x){
        // Filmic ACES approximation and linear -> sRGB transfer.
        const float v=std::clamp((x*(2.51f*x+0.03f))/(x*(2.43f*x+0.59f)+0.14f),0.0f,1.0f);
        return v<=0.0031308f?12.92f*v:1.055f*std::pow(v,1.0f/2.4f)-0.055f;
    };
    return {op(c.x),op(c.y),op(c.z)};
}
}

void Scene::quad(Vec3 a,Vec3 b,Vec3 c,Vec3 d,int material){
    triangles.emplace_back(a,b,c,material);
    triangles.emplace_back(a,c,d,material);
}
void Scene::box(Vec3 center,Vec3 half,float angle,int material){
    const float co=std::cos(angle),si=std::sin(angle);
    auto p=[&](float x,float y,float z){return center+Vec3{co*x+si*z,y,-si*x+co*z};};
    Vec3 p000=p(-half.x,-half.y,-half.z),p001=p(-half.x,-half.y,half.z);
    Vec3 p010=p(-half.x,half.y,-half.z),p011=p(-half.x,half.y,half.z);
    Vec3 p100=p(half.x,-half.y,-half.z),p101=p(half.x,-half.y,half.z);
    Vec3 p110=p(half.x,half.y,-half.z),p111=p(half.x,half.y,half.z);
    quad(p001,p101,p111,p011,material); // front
    quad(p100,p000,p010,p110,material); // back
    quad(p000,p001,p011,p010,material); // left
    quad(p101,p100,p110,p111,material); // right
    quad(p010,p011,p111,p110,material); // top
    quad(p000,p100,p101,p001,material); // bottom
}
void Scene::build_cornell_box(){
    triangles.clear();materials.clear();
    materials.push_back({{0.78f,0.78f,0.78f},{}}); // white
    materials.push_back({{0.75f,0.09f,0.075f},{}}); // red
    materials.push_back({{0.08f,0.63f,0.13f},{}}); // green
    materials.push_back({{0,0,0},{18,16,13}}); // emitter
    const int white=0,red=1,green=2,light=3;
    // Room: camera looks through open front (+Z).
    quad({-1,0,-1},{1,0,-1},{1,0,1},{-1,0,1},white); // floor
    quad({-1,2,1},{1,2,1},{1,2,-1},{-1,2,-1},white); // ceiling
    quad({-1,0,-1},{-1,2,-1},{1,2,-1},{1,0,-1},white); // back
    quad({-1,0,1},{-1,2,1},{-1,2,-1},{-1,0,-1},red); // left
    quad({1,0,-1},{1,2,-1},{1,2,1},{1,0,1},green); // right
    box({-0.42f,0.37f,-0.23f},{0.30f,0.37f,0.30f},-0.23f,white);
    box({0.42f,0.67f,-0.39f},{0.29f,0.67f,0.29f},0.29f,white);
    light_corner={-0.28f,1.99f,-0.67f};
    light_edge_u={0.56f,0,0}; light_edge_v={0,0,0.56f};
    light_area=length(cross(light_edge_u,light_edge_v));
    light_emission=materials[light].emission;
    // Normals of emitter quads point downward (camera sees the emitter).
    quad(light_corner, light_corner+light_edge_u,
         light_corner+light_edge_u+light_edge_v,light_corner+light_edge_v,light);
    build_bvh();
}
int Scene::build_node(int start,int end){
    int id=static_cast<int>(nodes_.size());
    nodes_.emplace_back();
    AABB b;
    for(int i=start;i<end;i++) b=merge(b,triangle_bounds(triangles[order_[i]]));
    nodes_[id].bounds=b;
    int count=end-start;
    if(count<=4){ nodes_[id].start=start;nodes_[id].count=count;return id; }
    Vec3 extent=b.hi-b.lo;
    int dim=(extent.x>extent.y && extent.x>extent.z)?0:(extent.y>extent.z?1:2);
    int mid=(start+end)/2;
    std::nth_element(order_.begin()+start,order_.begin()+mid,order_.begin()+end,
      [&](int a,int b){
        const Triangle &ta=triangles[a],&tb=triangles[b];
        return axis((ta.a+ta.b+ta.c)/3.0f,dim)<axis((tb.a+tb.b+tb.c)/3.0f,dim);
      });
    int left=build_node(start,mid),right=build_node(mid,end);
    nodes_[id].left=left;nodes_[id].right=right;
    return id;
}
void Scene::build_bvh(){
    nodes_.clear();order_.resize(triangles.size());
    std::iota(order_.begin(),order_.end(),0);
    if(!triangles.empty())build_node(0,static_cast<int>(triangles.size()));
}
bool Scene::intersect(const Ray& ray,float near_t,float far_t,Hit& result) const {
    bool found=false;
    if(nodes_.empty()) { // Useful for ad-hoc unit-test scenes.
        for(const Triangle& t:triangles){float distance;
            if(triangle_hit(t,ray,near_t,far_t,distance)){
                far_t=distance;found=true;result.t=distance;
                result.position=ray.origin+ray.direction*distance;
                result.normal=dot(t.normal,ray.direction)<0?t.normal:-t.normal;
                result.material=t.material;
            }
        }
        return found;
    }
    int stack[128];
    int stack_size=0; stack[stack_size++]=0;
    while(stack_size>0){
        int idx=stack[--stack_size];
        const BvhNode &node=nodes_[idx];
        if(!box_hit(node.bounds,ray,near_t,far_t))continue;
        if(node.count){
            for(int i=node.start;i<node.start+node.count;++i){
                const Triangle& t=triangles[order_[i]];
                float distance=0;
                if(triangle_hit(t,ray,near_t,far_t,distance)){
                    far_t=distance;found=true;result.t=distance;
                    result.position=ray.origin+ray.direction*distance;
                    result.normal=dot(t.normal,ray.direction)<0?t.normal:-t.normal;
                    result.material=t.material;
                }
            }
        } else {stack[stack_size++]=node.left;stack[stack_size++]=node.right;}
    }
    return found;
}
bool Scene::occluded(const Ray& ray,float near_t,float far_t) const {
    Hit h;return near_t<far_t && intersect(ray,near_t,far_t,h);
}

Image::Image(int w,int h):width(w),height(h),linear(static_cast<size_t>(w)*h){}
std::vector<uint8_t> Image::rgb8(float exposure)const{
    std::vector<uint8_t> out(linear.size()*3);
    for(size_t i=0;i<linear.size();++i){
        Vec3 v=tone(linear[i],exposure);
        out[i*3]=static_cast<uint8_t>(std::clamp(v.x*255.0f+0.5f,0.0f,255.0f));
        out[i*3+1]=static_cast<uint8_t>(std::clamp(v.y*255.0f+0.5f,0.0f,255.0f));
        out[i*3+2]=static_cast<uint8_t>(std::clamp(v.z*255.0f+0.5f,0.0f,255.0f));
    }
    return out;
}
bool Image::save_ppm(const std::string& filename,float exposure)const{
    std::ofstream f(filename,std::ios::binary);
    if(!f)return false;
    f<<"P6\n"<<width<<" "<<height<<"\n255\n";
    auto pixels=rgb8(exposure);
    f.write(reinterpret_cast<const char*>(pixels.data()),static_cast<std::streamsize>(pixels.size()));
    return static_cast<bool>(f);
}
bool Image::save_bmp(const std::string& filename,float exposure)const{
    std::ofstream f(filename,std::ios::binary);
    if(!f)return false;
    const uint32_t stride=static_cast<uint32_t>((width*3+3)&~3);
    const uint32_t bytes=stride*static_cast<uint32_t>(height);
    auto w16=[&](uint16_t v){f.put(static_cast<char>(v&255));f.put(static_cast<char>(v>>8));};
    auto w32=[&](uint32_t v){for(int i=0;i<4;i++)f.put(static_cast<char>((v>>(8*i))&255));};
    f.write("BM",2);w32(54+bytes);w32(0);w32(54); // BMP file header
    w32(40);w32(static_cast<uint32_t>(width));w32(static_cast<uint32_t>(height));
    w16(1);w16(24);w32(0);w32(bytes);w32(2835);w32(2835);w32(0);w32(0);
    auto pixels=rgb8(exposure);
    const uint8_t zero[3]={0,0,0};
    for(int y=height-1;y>=0;y--){
        for(int x=0;x<width;x++){
            size_t i=(static_cast<size_t>(y)*width+x)*3;
            const char bgr[]={static_cast<char>(pixels[i+2]),static_cast<char>(pixels[i+1]),static_cast<char>(pixels[i])};
            f.write(bgr,3);
        }
        f.write(reinterpret_cast<const char*>(zero),stride-static_cast<uint32_t>(width*3));
    }
    return static_cast<bool>(f);
}
bool Image::save(const std::string& filename,float exposure)const{
    const auto ext=filename.find_last_of('.');
    if(ext!=std::string::npos){
        std::string suffix=filename.substr(ext);
        for(auto& c:suffix)if(c>='A'&&c<='Z')c=static_cast<char>(c-'A'+'a');
        if(suffix==".bmp")return save_bmp(filename,exposure);
    }
    return save_ppm(filename,exposure);
}
Renderer::Renderer(Scene scene):scene_(std::move(scene)){}
void Renderer::render(const Settings& opt,Image& result,
                      const std::function<void(int,const Image&)>& on_pass,
                      const std::atomic<bool>* stop)const{
    if(opt.width<=0||opt.height<=0||opt.samples<=0||opt.max_bounces<=0)throw std::invalid_argument("Invalid render settings");
    if(result.width!=opt.width||result.height!=opt.height)throw std::invalid_argument("Image dimensions mismatch");
    std::fill(result.linear.begin(),result.linear.end(),Vec3{});
    const unsigned int hw=std::thread::hardware_concurrency();
    int threads=opt.threads>0?opt.threads:static_cast<int>(hw?hw:4);
    threads=std::clamp(threads,1,64);
    const int tile=16;
    const int tiles_x=(opt.width+tile-1)/tile,tiles_y=(opt.height+tile-1)/tile;
    const int jobs=tiles_x*tiles_y;
    const Vec3 eye=opt.camera_position,target=opt.camera_target;
    const Vec3 forward=normalize(target-eye),right=normalize(cross(forward,opt.camera_up));
    const Vec3 up=normalize(cross(right,forward));
    const float fov=std::tan(opt.fov_degrees*pi/360.0f);
    const float aspect=static_cast<float>(opt.width)/opt.height;
    // Process a short batch of samples per tile. Workers now start once per
    // progressive update rather than once per sample (128 spp => 16 batches).
    // Samples for each pixel stay in the same order, preserving results.
    const int passes_per_update=std::max(1,std::min(8,opt.samples/16));
    for(int batch=0;batch<opt.samples;){
        // Publish the first sample immediately, then process larger batches.
        const int batch_count=(batch==0 && on_pass)?1:passes_per_update;
        if(stop&&stop->load())break;
        const int sample_end=std::min(batch+batch_count,opt.samples);
        std::atomic<int> next{0};
        std::vector<std::thread> workers;
        workers.reserve(static_cast<size_t>(threads));
        for(int thread=0;thread<threads;++thread){
            workers.emplace_back([&]{
                int job;
                while((job=next.fetch_add(1))<jobs){
                    if(stop&&stop->load())break;
                    int tx=(job%tiles_x)*tile,ty=(job/tiles_x)*tile;
                    for(int y=ty;y<std::min(ty+tile,opt.height);++y){
                        if(stop&&stop->load())return;
                        for(int x=tx;x<std::min(tx+tile,opt.width);++x){
                            size_t index=static_cast<size_t>(y)*opt.width+x;
                            for(int sample=batch;sample<sample_end;++sample){
                                Rng rng(opt.seed^((index+1)*0x9e3779b97f4a7c15ull)^((static_cast<uint64_t>(sample)+1)*0xbf58476d1ce4e5b9ull));
                                float px=(2.0f*((x+rng.unit())/opt.width)-1.0f)*aspect*fov;
                                float py=(1.0f-2.0f*((y+rng.unit())/opt.height))*fov;
                                Ray ray{eye,normalize(forward+right*px+up*py)};
                                Vec3 color=trace(scene_,ray,rng,opt.max_bounces,opt.light_scale);
                                result.linear[index]+=(color-result.linear[index])/(sample+1.0f);
                            }
                        }
                    }
                }
            });
        }
        for(auto& t:workers)t.join();
        if(stop&&stop->load())break;
        if(on_pass)on_pass(sample_end,result);
        batch=sample_end;
    }
}
}