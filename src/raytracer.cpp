#include <iostream>
#include <sstream>
#include <cmath>
#include <fstream>
#include <vector>
#include <memory>
#include <limits>
#include <stdlib.h>
#include <iomanip>
#include <thread>
#include <mutex>
#include <exception>
#include <chrono>

#include <opencv2/opencv.hpp>


#include "container.h"
#include "camera.h"
#include "color.h"
#include "primitives.h"
#include "common.h"
#include "scene.h"

/** Some good resources ray tracing
 * Lecture about ray tracing: http://www.cs.uu.nl/docs/vakken/magr/2015-2016/index.html
 * url: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing
 * operator overloading: http://stackoverflow.com/a/4421719/1959528
 * 3D meshes: https://graphics.stanford.edu/data/3Dscanrep/
 * Idea: Create the cornell box as test scene: https://en.wikipedia.org/wiki/Cornell_box
 */



/** TODOs
 * - todo: Add material with different reflection models (refraction etc.)
 * - todo: Loading scene from file (xml, YAML etc.) -> I would prefer yaml
 * - todo: Soft-shadows
 * - todo: Area lights
 * - todo: depth of field
 * - todo: texture mapping
 * - todo: optimization: early pruning of objects which cannot be hit (kD-tree (spatial partitioning), BVH (object partitioning) etc.)
 * - todo: optimization: bounding box with fast intersection calculation around object (bounding box: sphere, box etc.)
 * - todo: optimization: do calculation on GPU
 * - todo: create scene to hold primitives, lights etc.
 * - todo: motion blur
 * - todo: global illumination: https://en.wikipedia.org/wiki/Global_illumination
 */

//using namespace std;
//using namespace raytracer;




class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    Float elapsed() const {
        return std::chrono::duration_cast<second_>
                (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<Float, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

bool processing{true};
const std::string winName{"image"};
typedef unsigned char ImageType;
Timer timer;
constexpr int MAX_DEPTH{10};
std::mutex RENDER_MUTEX;


struct Light {
    Vec3f pos;
    Color color;

    explicit Light(const Vec3f &pos) : Light{pos, color} {}
    Light(const Vec3f &pos, const Color& color) : pos{pos}, color{color} {}

};

void check_op_overloading() {
    Vec3f v1, v2{1, 2, 3};

    v1 += v2;
    std::cout << v1 << " == 1 2 3\n";
    std::cout << v1 * 2 << " == 2 4 6\n";
//    std::cout << v1+2 << " == 3 4 5\n";

    Color c1, c2{1, 2, 3};
    c1 += c2;
    std::cout << c1 << " == 1 2 3\n";
    std::cout << c1 * c2 << " == 1 4 9\n";
    std::cout << c1 * 2 << " == 2 4 6\n";

    const Vec3f e1{Vec3f{1, 0, 0}};
    const Vec3f e2{Vec3f{0, 1, 0}};
    const Vec3f e3{Vec3f{0, 0, 1}};
    const Vec3f cp{cross_product(e1, e2)};
    std::cout << "cross product: " << cp << " == 0 0 1\n";
    std::cout << "cross product angle should be 90 deg => 1): " << cos(cp.dotProduct(e1)) << " == 1 \n";


}


void
create_box(std::vector<std::shared_ptr<Primitive>>& objects) {
//    const int y_offset{-3};
//    const int x_offset{2};
//    const int z_offset{-9};
    const int y_offset{0};
    const int x_offset{0};
    const int z_offset{-5};

    const int x_width_half{1};
    const int y_width_half{1};
    const int z_width_half{1};

    /**
     * adopted from: http://stackoverflow.com/a/8142461/1959528
     *       6-------5
     *      /|      /|
     *    /  |     / |
     *   0-------3   |
     *   |   7---|---4
     *   |  /    |  /
     *   |/      |/
     *   1-------2
     */

    // front
    const Vec3f v0{x_offset - x_width_half, y_offset + y_width_half, z_offset + z_width_half};
    const Vec3f v1{x_offset - x_width_half, y_offset - y_width_half, z_offset + z_width_half};
    const Vec3f v2{x_offset + x_width_half, y_offset - y_width_half, z_offset + z_width_half};
    const Vec3f v3{x_offset + x_width_half, y_offset + y_width_half, z_offset + z_width_half};
    // back
    const Vec3f v4{x_offset + x_width_half, y_offset - y_width_half, z_offset - z_width_half};
    const Vec3f v5{x_offset + x_width_half, y_offset + y_width_half, z_offset - z_width_half};
    const Vec3f v6{x_offset - x_width_half, y_offset + y_width_half, z_offset - z_width_half};
    const Vec3f v7{x_offset - x_width_half, y_offset - y_width_half, z_offset - z_width_half};

//    const Color color{192.0 / 255, 155.0 / 255, 94.0 / 255};
    const Color color = Color::light_gray();
    Material mat{color};
    mat.ks = 0.4;
    mat.ka = 0.2;
    mat.kd = 0.5;
    mat.kr = 0.1;
    std::vector<Triangle> triangles;


    // front
    // 0---3
    // | \ |
    // 1---2
    triangles.emplace_back(Triangle{v0, v1, v2, mat});
    triangles.emplace_back(Triangle{v0, v2, v3, mat});
//    objects.push_back(make_shared<Triangle>( v0, v1, v2, color));
//    objects.push_back(make_shared<Triangle>( v0, v2, v3, color));
    // right
    // 3---5
    // | \ |
    // 2---4
    triangles.emplace_back(Triangle{v3, v2, v4, mat});
    triangles.emplace_back(Triangle{v3, v4, v5, mat});
//    objects.push_back(make_shared<Triangle>( v3, v2, v4, color));
//    objects.push_back(make_shared<Triangle>( v3, v4, v5, color));
    // back
    // 5---6
    // | \ |
    // 4---7
    triangles.emplace_back(Triangle{v5, v4, v7, mat});
    triangles.emplace_back(Triangle{v5, v7, v6, mat});
//    objects.push_back(make_shared<Triangle>( v5, v4, v7, color));
//    objects.push_back(make_shared<Triangle>( v5, v7, v6, color));
    // left
    // 6---0
    // | \ |
    // 7---1
    triangles.emplace_back(Triangle{v6, v1, v0, mat});
    triangles.emplace_back(Triangle{v6, v7, v1, mat});
//    objects.push_back(make_shared<Triangle>( v6, v7, v1, color));
//    objects.push_back(make_shared<Triangle>( v6, v1, v0, color));
    // top
    // 6---5
    // | \ |
    // 0---3
    triangles.emplace_back(Triangle{v6, v0, v3, mat});
    triangles.emplace_back(Triangle{v6, v3, v5, mat});
//    objects.push_back(make_shared<Triangle>( v6, v0, v3, color));
//    objects.push_back(make_shared<Triangle>( v6, v3, v5, color));
    // bottom
    // 4---7
    // | \ |
    // 2---1
    triangles.emplace_back(Triangle{v4, v2, v1, mat});
    triangles.emplace_back(Triangle{v4, v1, v7, mat});
//    objects.push_back(make_shared<Triangle>( v7, v1, v2, color));
//    objects.push_back(make_shared<Triangle>( v7, v1, v2, color));
    std::shared_ptr<Model> model{std::make_shared<Model>(mat, triangles)};
    objects.push_back(model);
}


void
preview(cv::Mat& img, unsigned int& finPix, unsigned int sumPix, int delay = 1000)
{
    int key{0};
    Float progress{0}, curElapsed{0}, lastElapsed{0};
    unsigned int curFinPix{0}, lastFinPix{0};
    bool printedProgress{false};
    const unsigned int progressInterval{5};
    while(processing)
    {
        cv::Mat tmpimg{img.clone()};
//        for (int n=0; n<numProg; ++n) {
//            curFinPix += finPixArr[n];
//            progress += (Float)(finPixArr[n])/sumPixArr[n];
//        }
        progress = (float) finPix / sumPix * 100;

        curElapsed = timer.elapsed();
        unsigned int curFinPix{finPix};
        const Float pixPerSec{((Float)(curFinPix - lastFinPix))/(curElapsed - lastElapsed)};
        lastFinPix = curFinPix;
        lastElapsed = curElapsed;
        const std::string text{std::to_string((unsigned int)progress) + "%; t: " + std::to_string((unsigned int)curElapsed)
                          + " s; " + std::to_string((unsigned int) pixPerSec) + " pix/s"};
        const int fontFace{CV_FONT_HERSHEY_SIMPLEX};
        const Float fontScale{0.8};
        const cv::Scalar color{0,255,0};
        const int thickness{1};
        int baseLine{0};
        const cv::Size textSize{cv::getTextSize(text, fontFace, fontScale, thickness+2, &baseLine)};
        const int x{10};
        const int y{10};
        const int border{12};
        const Float alpha{0.2};

        if (!((unsigned int)progress % progressInterval) && !printedProgress){
            std::cout << "... " << text << std::endl;
            printedProgress = true;
        }

        printedProgress = !((unsigned int)progress % progressInterval);

        // draw box;
        cv::Rect roi{x, y, textSize.width+border, textSize.height+border};
        cv::Mat foo{roi.height, roi.width, tmpimg.type(), cv::Scalar::all(130)};
        cv::Mat cropped{tmpimg(roi)};
        cv::putText(foo, text, cv::Point{(int)(border * 0.5), (int)(textSize.height+border * 0.5) -2}, fontFace, fontScale, color, thickness+2);
        cv::addWeighted(cropped, alpha, foo, 1-alpha, 0, cropped);
        cv::putText(cropped, text, cv::Point{(int)(border * 0.5), (int)(textSize.height+border * 0.5) -2}, fontFace, fontScale, color*0.5, thickness);

        cv::imshow(winName, tmpimg);
        key = cv::waitKey(delay);
        if ((key & 0xff) == 27) {
            std::cout << "... rendering canceled" << std::endl;
            exit(0);
        }
    }
}


Float calcDist(const std::vector<std::shared_ptr<Primitive>>& objects,
                const Ray& ray,
                Vec3f& hitNormal
) {
    Float dist{::std::numeric_limits<Float>::max()}, tmpdist;
    Vec3f tmpnormal;
    // get closest intersection
    for (const auto& o : objects) {
        if (o->intersect(ray, tmpdist, tmpnormal) && tmpdist < dist) {
            dist = tmpdist;
            hitNormal = tmpnormal;
        }

    }
    return dist;
}

Color
 trace(const std::vector<std::shared_ptr<Primitive>>& objects,
      const std::vector<Light>& lights,
      const Color& background,
      const Ray& ray,
      int depth) {
    Float dist{::std::numeric_limits<Float>::max()}, tmpdist;
    const Float bias{0.0001};   // bias origin of reflective/refractive/shadow ray a little into normal direction to adjust for precision problem
    Color color{background};
    Vec3f tmpnormal, hitNormal;
    std::shared_ptr<Primitive> cur_obj{nullptr};

    // get closest intersection
    for (const auto& object : objects) {
        if (object->intersect(ray, tmpdist, tmpnormal)) {
            if (tmpdist < dist) {
                dist = tmpdist;
                hitNormal = tmpnormal;
                cur_obj = object;
            }
        }
    }

    if (cur_obj == nullptr) {
        return color;
    }

    color = Color(0.0);
    const Material& curMaterial = cur_obj->material;
    const Vec3f hitPt{ray.getPoint(dist)};

    // shade pixel
    for (const auto& l : lights) {
        Vec3f lv{l.pos - hitPt};
        const Float ldist{lv.length()};
        lv.normalize();
        bool inShadow{false};

        // cast shadow ray and check if other object is blocking light
        const Ray shadow_ray{hitPt + hitNormal * bias, lv};
        for (const auto& o : objects) {
            const bool hit{o->intersect(shadow_ray, tmpdist, tmpnormal)};
            if (hit && tmpdist < ldist) {
                inShadow = true;
                break;
            }
        }
        if (inShadow) {
            continue;
        }


        // diffuse shading
        const Float cosPhi{
                std::max<Float>(hitNormal.dotProduct(lv), 0.0)};    // todo: check why we have to clip negative values
        color += curMaterial.color * l.color * curMaterial.kd * (cosPhi / (ldist * ldist));    // todo: change color to diffuse color

        // specular shading
        const Vec3f reflectRay{hitNormal * 2 * dotProduct(hitNormal,lv) - lv};
        const Float cosAlpha{
                std::max<Float>(reflectRay.dotProduct(lv), 0.0)};    // todo: check why we have to clip negative values
        color += l.color * curMaterial.ks * (pow(cosAlpha, curMaterial.specRefExp) / (ldist * ldist));     // todo: add reflective color here
//            color.clamp(0, 1);
    }

    ++depth;


    constexpr Float REFRACTIVE_INDEX_AIR{1.0};
    Vec3f refractNormal{hitNormal};
    Float cosIncedent{0}, cosTransmission{0}, sinSqrPhit{0};
    Float n1{0}, n2{0}, n{0};  // refractive index we come from (n1) and go to (n2)
    Float rCoef{0};    // reflective/refractive coefficient based on angle

    // fresnel equation
    if (curMaterial.refractive && depth<=MAX_DEPTH) {
        cosIncedent = ray.direction.dotProduct(hitNormal);     // is <0 if ray hits outside object hull (ray pointing towards object)
        if (cosIncedent < 0.0) {
            // ray from air to object
            n1 = REFRACTIVE_INDEX_AIR;   // assuming air other medium
            n2 = curMaterial.refractiveIdx;
            cosIncedent = -cosIncedent;
        } else {
            // ray from object to air (inside)
            n1 = curMaterial.refractiveIdx;
            n2 = REFRACTIVE_INDEX_AIR;   // assuming air other medium
            refractNormal = -refractNormal;
        }
        n = n1/n2;
        sinSqrPhit = n * n * (1 - cosIncedent * cosIncedent);    // if sinSqrPhit is greater than 1 it is not valid
        if (sinSqrPhit <= 1) {
            cosTransmission = sqrt(1-sinSqrPhit);
            const Float Rorth{(n1*cosIncedent - n2*cosTransmission)/(n1*cosIncedent + n2*cosTransmission)};
            const Float Rpar{ (n2*cosIncedent - n1*cosTransmission)/(n2*cosIncedent + n1*cosTransmission)};
            rCoef = (Rorth*Rorth + Rpar*Rpar) / 2;
        } else {
            rCoef = 1;
        }

    }

    // reflective shading recursive tracing
    Color colorReflect{0};
    if (curMaterial.reflective && depth <= MAX_DEPTH) {
        const Vec3f reflectDir{-hitNormal * 2 * dotProduct(hitNormal,ray.direction) + ray.direction};
        colorReflect = curMaterial.kr * trace(objects, lights, background, Ray{hitPt + hitNormal * bias, reflectDir}, depth);
    }
    // refractive shading recursive tracing
    Color colorRefract{0};
    if (curMaterial.refractive && depth <= MAX_DEPTH && sinSqrPhit <= 1) {
        // if sinSqrPhit > 1.0 we cannot find a transmission vector -> we have 'total internal reflection' (TRI) or critical angle
            // todo: handle tri and critical angle
        const Vec3f refractDir{ray.direction * n + refractNormal * (n * cosIncedent - cosTransmission)};
        colorRefract = curMaterial.kt * trace(objects, lights, background, Ray{hitPt - refractNormal * bias, refractDir}, depth);
    }


    color += curMaterial.color * curMaterial.ka;
    if (curMaterial.refractive) {
        color += rCoef * colorReflect + (1 - rCoef) * colorRefract;
    } else {
        color += colorReflect;
    }

    color.clamp(0, 1);



//        if (depth == MAX_DEPTH)
//        {
//            std::cout << "max depth reached: color: " << color << " factor r: " << rCoef << endl
//                 << "\tcurMaterial.reflective: " << (curMaterial.reflective ? "true" : "false") << endl
//                 << "\tcurMaterial.refractive: " << (curMaterial.refractive ? "true" : "false") << endl
//                 << "\tcolorReflect: " << colorReflect << endl
//                 << "\tcolorRefract: " << colorRefract << endl;
//        }

    return color;
}

void
render(ImageType* img, const unsigned int x_start, const unsigned int y_start, const unsigned int cH,
       const unsigned int cW, const Camera& camera, const std::vector<std::shared_ptr<Primitive>>& objects,
       const std::vector<Light>& lights, const Color& background, unsigned int& finPix,
       unsigned int& thread_count // only required when used in threads
) {
    const unsigned int W{camera.getImWidth()};
    const unsigned int H{camera.getImHeight()};

#if USE_OPENMP == 1
    #pragma omp parallel for schedule(dynamic,10)       // OpenMP
#endif
    for (unsigned int y = y_start; y < cH+y_start; ++y) {
        ImageType* img_ptr{img + 3 * (y*W + x_start)};
        for (unsigned int x = x_start; x < cW+x_start; ++x) {
//            const Ray ray{camera.castRay(x, y)};
            const std::vector<Ray> rays{camera.castRays(x,y)};
            const size_t numRays{rays.size()};
            Color px_color{0};

            // trace primary/camera ray
            for (const auto& ray : rays)
                px_color += trace(objects, lights, background, ray, 0) / numRays;
//            Color px_color{calcDist(objects, ray)};

            // Using Opencv bgr
            *(img_ptr++) = (ImageType) (px_color.b * 255);
            *(img_ptr++) = (ImageType) (px_color.g * 255);
            *(img_ptr++) = (ImageType) (px_color.r * 255);

            ++finPix;   // todo: make this thread-safe and efficient
        }
    }


#if USE_OPENMP == 1
    processing = false;     // notify main thread that processing is finished
#else
    {
        lock_guard<mutex> guard(RENDER_MUTEX);
        --thread_count;
    }
#endif
}

void
colorize_image_tile(cv::Mat img, int num, int x_start, int y_start, int pW, int pH)
{
    int baseline = 0;
    const int fontFace = CV_FONT_HERSHEY_SIMPLEX;
    const Float fontScale = 1;
    const int thickness = 2;
    const std::string text{"thread "+std::to_string(num)};
    const cv::Scalar bgColors[] = {cv::Scalar{0, 0, 255},
                                   cv::Scalar{0, 255, 0},
                                   cv::Scalar{255, 0, 0},
                                   cv::Scalar{255, 255, 0},
                                   cv::Scalar{255, 0, 255},
                                   cv::Scalar{0,255,255},
                                   cv::Scalar{125, 125, 125},
                                   cv::Scalar{0, 125, 255},
                                   cv::Scalar{125,255,255},
                                   cv::Scalar{125, 255, 0},
                                   cv::Scalar{255, 125, 0},
                                   cv::Scalar{255, 255, 125},
                                   cv::Scalar{255, 125, 255}};
    // color each part differently
    cv::Mat tile{img(cv::Rect{x_start, y_start, pW, pH})};
    tile = bgColors[num%13]*0.3;
    const cv::Size textSize{cv::getTextSize(text, fontFace, fontScale, thickness, &baseline)};
//    cv::putText(tile, text, cv::Point{pW / 2 - textSize.width/2, pH / 2}, fontFace, fontScale, cv::Scalar::all(255), thickness);
}


void
create_scene(std::vector<std::shared_ptr<Primitive>>& objects, std::vector<Light>& lights) {
    lights.emplace_back(Light{Vec3f{0, 3, -7.5}, Color::white() * 3});
//    lights.emplace_back(Light{Vec3f{0, 8, -9}, Color::white()*0.5});
    lights.emplace_back(Light{Vec3f{0, 0, -1}, Color::white() * 10});
//    lights.emplace_back(Light{Vec3f{5, -5, -2}, Color::white()*0.5});
//    lights.emplace_back(Light{Vec{-30,-20,1}});
    const Material glass(Color::glass(), 0.1, 0.1, 0.2, 0.8, 0.8, 8, 1.5, true, true);
    const Material glass2(Color::glass(), 0, 0, 0.2, 0.8, 0.2, 8, 1.5, true, true);
    const Material porcelain(Color::white(), 0.2, 0.5, 0.7, 0.3, 0, 8, 0, true);
    const Material mirror(Color::white(), 0, 0, 0, 1, 1, 8, 0, true);
    const Material m1(Color::light_gray(), 0.1, 0.7, 0.4, 0.3, 0, 8, 0, true);
    const Material m2(Color::white(), 0.1, 0.7, 0.4);

    objects.push_back(std::make_shared<Sphere>(Vec3f{0, 0, -8}, 1, Material(Color::red(), 0, 0, 0, 1, 1, 8, 0, true)));
    objects.push_back(std::make_shared<Sphere>(Vec3f{2, 0.25, -8}, 0.75, Material{Color{1, 1, 0}, 0.2, 0.7, 0}));
//    objects.push_back(make_shared<Sphere>(Vec3f{0, 1, -3}, 0.5, glass));
    objects.push_back(std::make_shared<Sphere>(Vec3f{-2.5, 2, -5}, 1, Material{Color{1, 0, 1}, 0.2, 0.5, 0.7}));

//    create_box(objects);
//    const string mesh_root{"/home/chris/shared/github/chris/raytracer/data/3d_meshes/"};
    const std::string mesh_root{"/home/chris/test/raytracer/data/3d_meshes/"};
    std::string bunny_res4_path{mesh_root + "bunny/reconstruction/bun_zipper_res4.ply"};
    std::string bunny_res2_path{mesh_root + "bunny/reconstruction/bun_zipper_res2.ply"};
    std::string bunny_path{mesh_root + "bunny/reconstruction/bun_zipper.ply"};
    std::shared_ptr<Model> bunny{Model::load_ply(bunny_path, porcelain, true)};     // glass bunny
//    shared_ptr<Model> bunny{Model::load_ply(bunny_path, Material(Color::white(), 0.2, 0.5, 0.8, 0.2))};
//    *bunny *= Mat3d::rotation(M_PI / 8, M_PI / 6, 0);
//    *bunny *= Mat3d::rotationX(M_PI/6);
//    *bunny *= Mat3d::rotationZ(M_PI/2);
//    *bunny *= Mat3d::rotationY(M_PI/4);
    bunny->scale(20);
    *bunny += Vec3f{-1.5, -3, -6};
    objects.emplace_back(bunny);


//    string draon_res4_path{mesh_root+"dragon_recon/dragon_vrip_res4.ply"};
//    string draon_path{mesh_root+"dragon_recon/dragon_vrip.ply"};
//    shared_ptr<Model> dragon{Model::load_ply(draon_path)};
//    dragon->scale(15);
//    dragon->translate(Vec3f{2, -2, -7.5});
//    objects.push_back(dragon);


//    string buddha_res4_path{mesh_root+"happy_recon/happy_vrip_res4.ply"};
//    string buddha_path{mesh_root+"happy_recon/happy_vrip.ply"};
//    shared_ptr<Model> buddha{Model::load_ply(buddha_path)};
//    buddha->scale(15);
//    buddha->translate(Vec3f{2, -4, -7.5});
//    buddha->material.ks = 0.9;
//    objects.push_back(buddha);


    // planes
    const int box_len{4};
    // back
    const Color wall_color{192.0 / 255, 155.0 / 255, 94.0 / 255};
    objects.push_back(std::make_shared<Plane>(0, 0, 1, box_len + 8, wall_color));
    // left
    objects.push_back(std::make_shared<Plane>(1, 0, 0, box_len, Color::red()));
    // right
//    objects.push_back(std::make_shared<Plane>(-1, 0, 0, box_len, Material{Color::green(), 0,0,0,1,0,8, 0, true}));   // mirror
    objects.push_back(std::make_shared<Plane>(-1, 0, 0, box_len, Color::green()));
    // bottom
    objects.push_back(std::make_shared<Plane>(0, 1, 0, box_len, Color::blue()));
    // top
    objects.push_back(std::make_shared<Plane>(0, -1, 0, box_len, wall_color));
    // behind camera
    objects.push_back(std::make_shared<Plane>(0, 0, -1, box_len, Color(1, 1, 0)));

//    s.radius = 10;
}


template< typename T >
struct array_deleter
{
    void operator ()( T const * p)
    {
        delete[] p;
    }
};

void thread_render(ImageType* img_ptr, const unsigned int imWidth, const unsigned int imHeight,
                   const std::vector<std::shared_ptr<Primitive>>& objects, const std::vector<Light>& lights,
                   const Camera& camera, const Color& background, unsigned int& finPix)
{

    // split image into tiles (2x4)
    const unsigned int nXTiles{20};
    const unsigned int nYTiles{20};
    const unsigned int tileWidth{imWidth/nXTiles};
    const unsigned int tileHeight{imHeight/nYTiles};

    const unsigned int max_threads{std::thread::hardware_concurrency()};   // max: num available threads
    std::cout << "... starting " << max_threads << " threads" << std::endl;
    std::vector<std::thread> threads;
    unsigned int x_start{0};
    unsigned int y_start{0};
    unsigned int ctr{0};
    unsigned int thread_count{0};
    while (true) {
        {
            std::lock_guard<std::mutex> guard(RENDER_MUTEX);
            threads.emplace_back(render, img_ptr, x_start, y_start, tileHeight, tileWidth, std::ref(camera),
                                     std::ref(objects),
                                     lights, background,
                                     std::ref(finPix),
                                     std::ref(thread_count));
            ++thread_count;
        }
//        colorize_image_tile(img, ctr++, x_start, y_start, tileWidth, tileHeight);
        x_start += tileWidth;
        if (x_start >= imWidth) {
            x_start = 0;
            y_start += tileHeight;
            if (y_start >= imHeight)
                break;
        }
        // wait until new threads can be started
        while (thread_count >= max_threads) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // wait for other threads to finish
    std::cout << "... waiting for threads to finish " << std::endl;
    for (auto& t : threads) {
        if (t.joinable())
            t.join();
    }

    processing = false;     // notify main thread that processing is finished
}

int
main(int argc, char** argv) {
    std::string outFilename{"raytracer.png"};
    if(argc > 1)
    {
        if (std::string(argv[1]) == "-o") {
            outFilename = argv[2];
        }
    }

    std::cout << "... start ray tracer" << std::endl;
#if MT_TRIANGLE_INTERSECT == 1
    std::cout << "... using Moeller-Trumbore algorithm for triangle intersection calculation" << std::endl;
#endif
    std::cout << "... write to file: " << outFilename << std::endl;

#if USE_OPENMP == 1
    std::cout << "... using OpenMP for parallelization" << std::endl;
#else
    std::cout << "... using c++ 11 threads for parallelization" << std::endl;
#endif
//    check_op_overloading();

    // resolution has to be even
    constexpr unsigned int downScale{1};
    constexpr unsigned int imHeight{600/downScale};
    constexpr unsigned int imWidth{800/downScale};

    std::shared_ptr<ImageType> img_ptr{new ImageType[imWidth*imHeight*3], array_deleter<ImageType>()};
    memset(img_ptr.get(), 0, sizeof(ImageType)*imWidth*imHeight*3);
    cv::Mat img{imHeight, imWidth, CV_8UC3, img_ptr.get()};
    cv::namedWindow(winName, CV_WINDOW_AUTOSIZE);

//    constexpr unsigned int H = 250;
//    constexpr unsigned int W = 350;
    constexpr unsigned int MAX_VAL = 255;
    constexpr Float ASPECT_RATIO = (Float) imWidth / imHeight;
    constexpr Float FOV = 60;

    Color background{0, 0.5, 0.5};
    Camera camera{Vec3f{0,0,0}, Vec3f{0,1,0}, Vec3f{0,0,-1}, ASPECT_RATIO, FOV, imWidth, imHeight};
    camera.rotate(0,0,M_PI/16);
//    camera.rotate(0,0,0);
//    camera.move(Vec3f{-0.5,0,-0.25});

    std::vector<std::shared_ptr<Primitive>> objects;
    std::vector<Light> lights;
    create_scene(objects, lights);
    unsigned int finPix{0};

    // split image into tiles (2x4)
    const unsigned int nXTiles{20};
    const unsigned int nYTiles{20};
    const unsigned int tileWidth{imWidth/nXTiles};
    const unsigned int tileHeight{imHeight/nYTiles};

    timer.reset();
    // starting thread to show progress
//    thread thread_show{preview, std::ref(img), std::ref(finPix), imWidth*imHeight, 200};

#if USE_OPENMP == 0
    // start threads
    thread renderThread{thread_render, img_ptr, imWidth, imHeight, std::ref(objects), std::ref(lights), std::ref(camera),
                        std::ref(background), std::ref(finPix)};
#else
    unsigned int thread_count = 0;
    std::thread renderThread{render, img_ptr.get(), 0, 0, imHeight, imWidth, std::ref(camera), std::ref(objects), std::ref(lights),
                        std::ref(background), std::ref(finPix), std::ref(thread_count)};
#endif

    preview(img, finPix, imWidth*imHeight, 200);

    std::cout << "... finished rendering (" << timer.elapsed() << "s)" << std::endl;
    std::cout << "... write image " << outFilename << std::endl;
    cv::imwrite(outFilename, img);

    cv::imshow(winName, img);
    cv::waitKey();
    if (renderThread.joinable())
        renderThread.join();
//    delete[] img_ptr;
}
