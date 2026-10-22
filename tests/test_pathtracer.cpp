#include "pathtracer.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main(){
    using namespace mpt;
    assert(std::abs(dot(Vec3{1,2,3},Vec3{4,5,6})-32)<1e-6f);
    assert(std::abs(length(Vec3{3,4,0})-5)<1e-6f);
    Scene s;s.build_cornell_box();
    assert(s.triangles.size()>=30);
    assert(s.bvh_node_count()>1);
    Hit hit;
    // The camera center ray must strike the back wall or an inner box.
    bool ok=s.intersect({{0,1,3.35f},{0,0,-1}},0.0001f,1000,hit);
    assert(ok && hit.t>0 && hit.material>=0);
    Hit miss;
    assert(!s.intersect({{0,3,3},{0,1,0}},0.0001f,1000,miss));
    Settings opts;opts.width=16;opts.height=16;opts.samples=2;opts.max_bounces=5;opts.threads=2;
    Image image(16,16);
    Renderer renderer(s);
    int count=0;
    renderer.render(opts,image,[&](int sample,const Image&){assert(sample>=1);++count;});
    assert(count>=1);
    bool nonzero=false;
    for(auto c:image.linear){
        assert(std::isfinite(c.x)&&std::isfinite(c.y)&&std::isfinite(c.z));
        assert(c.x>=0&&c.y>=0&&c.z>=0);
        if(c.x+c.y+c.z>0.001f)nonzero=true;
    }
    assert(nonzero);
    assert(image.rgb8(1.0f).size()==16u*16u*3u);
    std::cout<<"All MiniPathTracer tests passed\n";
}