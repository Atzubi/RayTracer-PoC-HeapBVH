//
// Created by Sebastian on 20.02.2019.
//

#ifndef HEAP_BVH_TESTRENDERER_H
#define HEAP_BVH_TESTRENDERER_H

#include "HBVH.h"
#include <SFML/Graphics.hpp>
#include <bitset>

sf::RenderWindow window;

struct Ray {
    Vector orig, dir;
};

struct IntersectionInfo {
    Vector normal;
    Vector pos;
    double_t dist;
};

struct Pixel {
    uint8_t red, blue, green;
};

Ray createRay(int resX, int resY, int id, double_t fov, Vector pos, Vector dir) {
    Vector rayDir;
    Ray ray;
    rayDir.dim[0] = id % resY - resY / 2;
    rayDir.dim[1] = id / resX - resX / 2;
    rayDir.dim[2] = fov;

    double_t dist = sqrt(rayDir.dim[0] * rayDir.dim[0] + rayDir.dim[1] * rayDir.dim[1] + rayDir.dim[2] * rayDir.dim[2]);

    rayDir.dim[0] /= dist;
    rayDir.dim[1] /= dist;
    rayDir.dim[2] /= dist;

    Vector z;
    z.dim[0] = dir.dim[0];
    z.dim[1] = dir.dim[1];
    z.dim[2] = dir.dim[2];

    Vector x;
    x.dim[0] = dir.dim[2];
    x.dim[1] = 0;
    x.dim[2] = -dir.dim[0];

    dist = sqrt(x.dim[0] * x.dim[0] + x.dim[1] * x.dim[1] + x.dim[2] * x.dim[2]);

    x.dim[0] /= dist;
    x.dim[1] /= dist;
    x.dim[2] /= dist;

    Vector y;
    y.dim[0] = dir.dim[0] * dir.dim[1];
    y.dim[1] = -dir.dim[0] * dir.dim[0] - dir.dim[2] * dir.dim[2];
    y.dim[2] = dir.dim[1] * dir.dim[2];

    dist = sqrt(y.dim[0] * y.dim[0] + y.dim[1] * y.dim[1] + y.dim[2] * y.dim[2]);

    y.dim[0] /= dist;
    y.dim[1] /= dist;
    y.dim[2] /= dist;

    ray.dir.dim[0] = rayDir.dim[0] * x.dim[0] + rayDir.dim[1] * x.dim[1] + rayDir.dim[2] * x.dim[2];
    ray.dir.dim[1] = rayDir.dim[0] * y.dim[0] + rayDir.dim[1] * y.dim[1] + rayDir.dim[2] * y.dim[2];
    ray.dir.dim[2] = rayDir.dim[0] * z.dim[0] + rayDir.dim[1] * z.dim[1] + rayDir.dim[2] * z.dim[2];

    dist = sqrt(ray.dir.dim[0] * ray.dir.dim[0] + ray.dir.dim[1] * ray.dir.dim[1] + ray.dir.dim[2] * ray.dir.dim[2]);

    ray.dir.dim[0] /= dist;
    ray.dir.dim[1] /= dist;
    ray.dir.dim[2] /= dist;

    ray.orig = pos;
    return ray;
}

bool rayBoxIntersection(Vector min, Vector max, Ray r) {
    double_t tmin = (min.dim[0] - r.orig.dim[0]) / r.dir.dim[0];
    double_t tmax = (max.dim[0] - r.orig.dim[0]) / r.dir.dim[0];

    if (tmin < 0 && tmax < 0) return false;
    if (tmin > tmax) swap(&tmin, &tmax);

    double_t tymin = (min.dim[1] - r.orig.dim[1]) / r.dir.dim[1];
    double_t tymax = (max.dim[1] - r.orig.dim[1]) / r.dir.dim[1];

    if (tymin < 0 && tymax < 0) return false;
    if (tymin > tymax) swap(&tymin, &tymax);

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    if (tymin > tmin)
        tmin = tymin;

    if (tymax < tmax)
        tmax = tymax;

    double_t tzmin = (min.dim[2] - r.orig.dim[2]) / r.dir.dim[2];
    double_t tzmax = (max.dim[2] - r.orig.dim[2]) / r.dir.dim[2];

    if (tzmin < 0 && tzmax < 0) return false;
    if (tzmin > tzmax) swap(&tzmin, &tzmax);

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    if (tzmin > tmin)
        tmin = tzmin;

    if (tzmax < tmax)
        tmax = tzmax;

    return true;
}

bool intersectPrimitive(IntersectionInfo *info, Primitive primitive, Ray ray) {
    Vector e1, e2, pvec, qvec, tvec;
    double_t epsilon = 0.000001f;

    e1.dim[0] = primitive.v2.dim[0] - primitive.v1.dim[0];
    e1.dim[1] = primitive.v2.dim[1] - primitive.v1.dim[1];
    e1.dim[2] = primitive.v2.dim[2] - primitive.v1.dim[2];

    e2.dim[0] = primitive.v3.dim[0] - primitive.v1.dim[0];
    e2.dim[1] = primitive.v3.dim[1] - primitive.v1.dim[1];
    e2.dim[2] = primitive.v3.dim[2] - primitive.v1.dim[2];

    pvec.dim[0] = ray.dir.dim[1] * e2.dim[2] - ray.dir.dim[2] * e2.dim[1];
    pvec.dim[1] = ray.dir.dim[2] * e2.dim[0] - ray.dir.dim[0] * e2.dim[2];
    pvec.dim[2] = ray.dir.dim[0] * e2.dim[1] - ray.dir.dim[1] * e2.dim[0];

    double_t dist = sqrt(
            ray.dir.dim[0] * ray.dir.dim[0] + ray.dir.dim[1] * ray.dir.dim[1] + ray.dir.dim[2] * ray.dir.dim[2]);

    ray.dir.dim[0] /= dist;
    ray.dir.dim[1] /= dist;
    ray.dir.dim[2] /= dist;

    //NORMALIZE(pvec);
    double_t det = pvec.dim[0] * e1.dim[0] + pvec.dim[1] * e1.dim[1] + pvec.dim[2] * e1.dim[2];

    if (det < epsilon && det > -epsilon) {
        return false;
    }

    double_t invDet = 1.0f / det;

    tvec.dim[0] = ray.orig.dim[0] - primitive.v1.dim[0];
    tvec.dim[1] = ray.orig.dim[1] - primitive.v1.dim[1];
    tvec.dim[2] = ray.orig.dim[2] - primitive.v1.dim[2];

    // NORMALIZE(tvec);
    double_t u = invDet * (tvec.dim[0] * pvec.dim[0] + tvec.dim[1] * pvec.dim[1] + tvec.dim[2] * pvec.dim[2]);

    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    qvec.dim[0] = tvec.dim[1] * e1.dim[2] - tvec.dim[2] * e1.dim[1];
    qvec.dim[1] = tvec.dim[2] * e1.dim[0] - tvec.dim[0] * e1.dim[2];
    qvec.dim[2] = tvec.dim[0] * e1.dim[1] - tvec.dim[1] * e1.dim[0];

    // NORMALIZE(qvec);
    double_t v = invDet * (qvec.dim[0] * ray.dir.dim[0] + qvec.dim[1] * ray.dir.dim[1] + qvec.dim[2] * ray.dir.dim[2]);

    if (v < 0.0f || u + v > 1.0f) {

        return false;
    }

    info->pos.dim[0] = primitive.v1.dim[0] + u * e2.dim[0] + v * e1.dim[0];
    info->pos.dim[1] = primitive.v1.dim[1] + u * e2.dim[1] + v * e1.dim[1];
    info->pos.dim[2] = primitive.v1.dim[2] + u * e2.dim[2] + v * e1.dim[2];

    info->dist = sqrt((ray.orig.dim[0] - info->pos.dim[0]) * (ray.orig.dim[0] - info->pos.dim[0]) +
                      (ray.orig.dim[1] - info->pos.dim[1]) * (ray.orig.dim[1] - info->pos.dim[1]) +
                      (ray.orig.dim[2] - info->pos.dim[2]) * (ray.orig.dim[2] - info->pos.dim[2]));

    info->normal.dim[0] = (primitive.v1.dim[1] - primitive.v2.dim[1]) * (primitive.v1.dim[2] - primitive.v3.dim[2]) -
                          (primitive.v1.dim[2] - primitive.v2.dim[2]) * (primitive.v1.dim[1] - primitive.v3.dim[1]);
    info->normal.dim[1] = (primitive.v1.dim[2] - primitive.v2.dim[2]) * (primitive.v1.dim[0] - primitive.v3.dim[0]) -
                          (primitive.v1.dim[0] - primitive.v2.dim[0]) * (primitive.v1.dim[2] - primitive.v3.dim[2]);
    info->normal.dim[2] = (primitive.v1.dim[0] - primitive.v2.dim[0]) * (primitive.v1.dim[1] - primitive.v3.dim[1]) -
                          (primitive.v1.dim[1] - primitive.v2.dim[1]) * (primitive.v1.dim[0] - primitive.v3.dim[0]);

    double_t length = sqrt(info->normal.dim[0] * info->normal.dim[0] + info->normal.dim[1] * info->normal.dim[1] +
                           info->normal.dim[2] * info->normal.dim[2]);

    info->normal.dim[0] /= length;
    info->normal.dim[1] /= length;
    info->normal.dim[2] /= length;

    return true;
}

IntersectionInfo testIntersection(Ray ray, HBVH *tree, Primitive *triangles) {
    IntersectionInfo info;
    info.pos.dim[0] = 0;
    info.pos.dim[1] = 0;
    info.pos.dim[2] = 0;
    info.normal.dim[0] = 0;
    info.normal.dim[1] = 0;
    info.normal.dim[2] = 0;
    info.dist = std::numeric_limits<double_t>::max();
    uint64_t miniStack = 0;
    uint64_t stackPointer = 1;
    uint64_t pos = 0;
    Vector min, max;
    min.dim[0] = *((double_t *) &(tree[0]));
    min.dim[1] = *((double_t *) &(tree[1]));
    min.dim[2] = *((double_t *) &(tree[2]));
    max.dim[0] = *((double_t *) &(tree[3]));
    max.dim[1] = *((double_t *) &(tree[4]));
    max.dim[2] = *((double_t *) &(tree[5]));

    if (rayBoxIntersection(min, max, ray)) {
        stackPointer = stackPointer << 1;
        pos++;

        //int c = 0;
        while (stackPointer > 1) {
            //c++;
            //std::bitset<64> sp(stackPointer);
            //std::bitset<64> ms(miniStack);
            if ((miniStack & 1) != 0) {
                if ((stackPointer & miniStack) != 0) {
                    stackPointer = stackPointer >> 1;
                    pos = (pos - 1) / 2;

                    Vector minB = min, maxB = max;

                    min.dim[0] = (tree[6 + pos].node.bBox.x1 * maxB.dim[0] +
                                  (tree[6 + pos].node.bBox.x2 - 256) * minB.dim[0]) /
                                 (tree[6 + pos].node.bBox.x1 + tree[6 + pos].node.bBox.x2 - 256);
                    min.dim[1] = (tree[6 + pos].node.bBox.y1 * maxB.dim[1] +
                                  (tree[6 + pos].node.bBox.y2 - 256) * minB.dim[1]) /
                                 (tree[6 + pos].node.bBox.y1 + tree[6 + pos].node.bBox.y2 - 256);
                    min.dim[2] = (tree[6 + pos].node.bBox.z1 * maxB.dim[2] +
                                  (tree[6 + pos].node.bBox.z2 - 256) * minB.dim[2]) /
                                 (tree[6 + pos].node.bBox.z1 + tree[6 + pos].node.bBox.z2 - 256);
                    max.dim[0] = (tree[6 + pos].node.bBox.x1 * maxB.dim[0] + tree[6 + pos].node.bBox.x2 * minB.dim[0] -
                                  256 * maxB.dim[0]) / (tree[6 + pos].node.bBox.x1 + tree[6 + pos].node.bBox.x2 - 256);
                    max.dim[1] = (tree[6 + pos].node.bBox.y1 * maxB.dim[1] + tree[6 + pos].node.bBox.y2 * minB.dim[1] -
                                  256 * maxB.dim[1]) / (tree[6 + pos].node.bBox.y1 + tree[6 + pos].node.bBox.y2 - 256);
                    max.dim[2] = (tree[6 + pos].node.bBox.z1 * maxB.dim[2] + tree[6 + pos].node.bBox.z2 * minB.dim[2] -
                                  256 * maxB.dim[2]) / (tree[6 + pos].node.bBox.z1 + tree[6 + pos].node.bBox.z2 - 256);

                    continue;
                } else {
                    miniStack = miniStack ^ 1;
                    miniStack = miniStack | stackPointer;
                    pos = pos + 1;
                }
            } else {
                miniStack = ~((~miniStack) | stackPointer);
            }

            Vector minBuffer = min, maxBuffer = max;
            min.dim[0] =
                    minBuffer.dim[0] + tree[6 + pos].node.bBox.x1 * ((maxBuffer.dim[0] - minBuffer.dim[0]) / (256));
            min.dim[1] =
                    minBuffer.dim[1] + tree[6 + pos].node.bBox.y1 * ((maxBuffer.dim[1] - minBuffer.dim[1]) / (256));
            min.dim[2] =
                    minBuffer.dim[2] + tree[6 + pos].node.bBox.z1 * ((maxBuffer.dim[2] - minBuffer.dim[2]) / (256));
            max.dim[0] =
                    maxBuffer.dim[0] - tree[6 + pos].node.bBox.x2 * ((maxBuffer.dim[0] - minBuffer.dim[0]) / (256));
            max.dim[1] =
                    maxBuffer.dim[1] - tree[6 + pos].node.bBox.y2 * ((maxBuffer.dim[1] - minBuffer.dim[1]) / (256));
            max.dim[2] =
                    maxBuffer.dim[2] - tree[6 + pos].node.bBox.z2 * ((maxBuffer.dim[2] - minBuffer.dim[2]) / (256));

            if (rayBoxIntersection(min, max, ray)) {
                /*info.pos.dim[0] = info.pos.dim[0] + 1 > 255 ? 255 : info.pos.dim[0] + 1;
                info.pos.dim[1] = info.pos.dim[1] + 1 > 255 ? 255 : info.pos.dim[1] + 1;
                info.pos.dim[2] = info.pos.dim[2] + 1 > 255 ? 255 : info.pos.dim[2] + 1;*/
                if (tree[6 + pos].node.flags == 1) {
                    IntersectionInfo infoBuffer = info;
                    if (intersectPrimitive(&info, triangles[tree[6 + pos * 2 + 1].primitivePointer], ray) ||
                        intersectPrimitive(&info, triangles[tree[6 + pos * 2 + 2].primitivePointer], ray)) {
                        /*info.pos.dim[0] = 255;
                        info.pos.dim[1] = 255;
                        info.pos.dim[2] = 0;
                        return info;*/
                        if (info.dist >= infoBuffer.dist)
                            info = infoBuffer;
                    }
                    miniStack = miniStack | 1;

                    min = minBuffer;
                    max = maxBuffer;
                } else {
                    stackPointer = stackPointer << 1;
                    pos = pos * 2 + 1;
                }
            } else {
                miniStack = miniStack | 1;

                min = minBuffer;
                max = maxBuffer;
            }
        }
    }
    return info;
}

Pixel shade(IntersectionInfo info, Vector cPos, Vector cDir) {
    Vector light;
    light.dim[0] = 10;
    light.dim[1] = 10;
    light.dim[2] = 0;

    double_t diffuse = 1, specular = 0.5, exponent = 8, ambient = 0.1;

    Pixel pixel;
    pixel.red = 0;
    pixel.green = 0;
    pixel.blue = 0;

    if (info.dist == std::numeric_limits<double_t>::max()) return pixel;

    Vector n, l, v, r;
    double_t nl;

    l.dim[0] = light.dim[0];
    l.dim[1] = light.dim[1];
    l.dim[2] = light.dim[2];

    n = info.normal;

    v.dim[0] = -1.0 * (info.pos.dim[0] - cPos.dim[0]);
    v.dim[1] = -1.0 * (info.pos.dim[1] - cPos.dim[1]);
    v.dim[2] = -1.0 * (info.pos.dim[2] - cPos.dim[2]);


    double_t length = sqrt(l.dim[0] * l.dim[0] + l.dim[1] * l.dim[1] + l.dim[2] * l.dim[2]);
    l.dim[0] /= length;
    l.dim[1] /= length;
    l.dim[2] /= length;

    length = sqrt(n.dim[0] * n.dim[0] + n.dim[1] * n.dim[1] + n.dim[2] * n.dim[2]);
    n.dim[0] /= length;
    n.dim[1] /= length;
    n.dim[2] /= length;

    length = sqrt(v.dim[0] * v.dim[0] + v.dim[1] * v.dim[1] + v.dim[2] * v.dim[2]);
    v.dim[0] /= length;
    v.dim[1] /= length;
    v.dim[2] /= length;

    nl = fmax(n.dim[0] * l.dim[0] + n.dim[1] * l.dim[1] + n.dim[2] * l.dim[2], 0);

    r.dim[0] = (2 * nl * n.dim[0]) - l.dim[0];
    r.dim[1] = (2 * nl * n.dim[1]) - l.dim[1];
    r.dim[2] = (2 * nl * n.dim[2]) - l.dim[2];

    length = sqrt(r.dim[0] * r.dim[0] + r.dim[1] * r.dim[1] + r.dim[2] * r.dim[2]);
    r.dim[0] /= length;
    r.dim[1] /= length;
    r.dim[2] /= length;

    double_t dot = fmax(v.dim[0] * r.dim[0] + v.dim[1] * r.dim[1] + v.dim[2] * r.dim[2], 0);

    pixel.red = (uint8_t) fmin((ambient + diffuse * nl + specular * powf(dot, exponent)) * 255, 255);
    pixel.green = (uint8_t) fmin((ambient + diffuse * nl + specular * powf(dot, exponent)) * 255, 255);
    pixel.blue = (uint8_t) fmin((ambient + diffuse * nl + specular * powf(dot, exponent)) * 255, 255);

    return pixel;
}

void display(Pixel *image, int resX, int resY) {
    sf::Image test;
    sf::Uint8 pixels[resX * resY * 4];

    for (int i = 0; i < resX; i++) {
        for (int j = 0; j < resY; j++) {
            pixels[(i + j * resX) * 4 + 0] = image[i + j * resX].red;
            pixels[(i + j * resX) * 4 + 1] = image[i + j * resX].green;
            pixels[(i + j * resX) * 4 + 2] = image[i + j * resX].blue;
            pixels[(i + j * resX) * 4 + 3] = 255;
        }
    }

    test.create(resX, resY, pixels);
    sf::Texture texture;
    texture.loadFromImage(test);
    sf::Sprite sprite;
    sprite.setTexture(texture);
    window.draw(sprite);
}

void workerTest(Pixel *image, int id, int resX, int resY, Vector cameraPos, Vector cameraDir, HBVH *tree,
                Primitive *triangles, int threadcount) {
    for (int j = 0; j < (resX * resY) / threadcount; j++) {
        image[j + id * ((resX * resY) / threadcount)] = shade(
                testIntersection(createRay(resX, resY, j + id * ((resX * resY) / threadcount), 1500, cameraPos, cameraDir), tree,
                                 triangles), cameraPos, cameraDir);
    }

    /*for (int j = 0; j < (resX * resY) / threadcount; j++) {
        image[j + id * ((resX * resY) / threadcount)] = shade(
                testIntersection(viewspace[j + id * ((resX * resY) / threadcount)], tree, triangles), cameraPos,
                cameraDir);
    }*/
}

void render(HBVH *tree, Primitive *triangles, Vector cameraPos, Vector cameraDir) {
    int resX = 1000, resY = 1000;
    int mposX = 0, mposY = 0;
    int threadcount = 128;

    Pixel *image = new Pixel[resX * resY];

    /*Ray *viewspace = new Ray[resX * resY];

    for (int i = 0; i < resX * resY; i++) {
        viewspace[i] = createRay(resX, resY, i, 1500, cameraPos, cameraDir);
    }*/

    window.create(sf::VideoMode(resX, resY), "OMEGALUL");

    int frames = 0;
    auto reference = std::chrono::high_resolution_clock::now();
    while (window.isOpen()) {
        window.clear(sf::Color(255, 255, 255));

        sf::Event event;
        while (window.pollEvent(event)) {
            switch (event.type) {
                case sf::Event::Closed:
                    window.close();
                    break;
                case sf::Event::KeyPressed:
                    switch (event.key.code) {
                        case sf::Keyboard::Down:
                            cameraPos.dim[1] -= 0.1;
                            break;
                        case sf::Keyboard::Up:
                            cameraPos.dim[1] += 0.1;
                            break;
                        case sf::Keyboard::Left:
                            cameraPos.dim[0] -= 0.1;
                            break;
                        case sf::Keyboard::Right:
                            cameraPos.dim[0] += 0.1;
                            break;
                        case sf::Keyboard::Add:
                            cameraPos.dim[2] += 0.1;
                            break;
                        case sf::Keyboard::Subtract:
                            cameraPos.dim[2] -= 0.1;
                            break;
                        case sf::Keyboard::Escape:
                            window.close();
                            break;
                        case sf::Keyboard::W:
                            cameraPos.dim[0] -= cameraDir.dim[0];
                            cameraPos.dim[1] += cameraDir.dim[1];
                            cameraPos.dim[2] += cameraDir.dim[2];
                            break;
                        case sf::Keyboard::A:
                            cameraPos.dim[0] -= cameraDir.dim[2] / 10.f;
                            cameraPos.dim[2] -= cameraDir.dim[0] / 10.f;
                            break;
                        case sf::Keyboard::S:
                            cameraPos.dim[0] += cameraDir.dim[0];
                            cameraPos.dim[1] -= cameraDir.dim[1];
                            cameraPos.dim[2] -= cameraDir.dim[2];
                            break;
                        case sf::Keyboard::D:
                            cameraPos.dim[0] += cameraDir.dim[2] / 10.f;
                            cameraPos.dim[2] += cameraDir.dim[0] / 10.f;
                            break;
                        default:
                            break;
                    }
                    break;
                case sf::Event::MouseMoved: {
                    if (sf::Mouse::getPosition().x == resX / 2 && sf::Mouse::getPosition().y == resY / 2) break;

                    Vector cb = cameraDir;

                    cameraDir.dim[0] = -cb.dim[2] * std::cos((sf::Mouse::getPosition().y - resX / 2.f) / (resX * 10)) *
                                       std::sin((sf::Mouse::getPosition().x - resX / 2.f) / (resX * 10)) -
                                       cb.dim[1] * std::sin((sf::Mouse::getPosition().y - resX / 2.f) / (resX * 10)) *
                                       std::sin((sf::Mouse::getPosition().x - resX / 2.f) / (resX * 10)) +
                                       cb.dim[0] * std::cos((sf::Mouse::getPosition().x - resX / 2.f) / (resX * 10));

                    cameraDir.dim[1] = cb.dim[1] * std::cos((sf::Mouse::getPosition().y - resX / 2.f) / (resX * 10)) -
                                       cb.dim[2] * std::sin((sf::Mouse::getPosition().y - resX / 2.f) / (resX * 10));

                    cameraDir.dim[2] = cb.dim[2] * std::cos((sf::Mouse::getPosition().y - resX / 2.f) / (resX * 10)) *
                                       std::cos((sf::Mouse::getPosition().x - resX / 2.f) / (resX * 10)) +
                                       cb.dim[1] * std::sin((sf::Mouse::getPosition().y - resX / 2.f) / (resX * 10)) *
                                       std::cos((sf::Mouse::getPosition().x - resX / 2.f) / (resX * 10)) +
                                       cb.dim[0] * std::sin((sf::Mouse::getPosition().x - resX / 2.f) / (resX * 10));

                    double_t dist = sqrt(cameraDir.dim[0] * cameraDir.dim[0] + cameraDir.dim[1] * cameraDir.dim[1] +
                                         cameraDir.dim[2] * cameraDir.dim[2]);

                    cameraDir.dim[0] /= dist;
                    cameraDir.dim[1] /= dist;
                    cameraDir.dim[2] /= dist;

                    sf::Mouse::setPosition(sf::Vector2i(resX / 2, resY / 2));

                    break;
                }
                default:
                    break;
            }
        }

        std::thread worker[threadcount];

        for (int i = 0; i < threadcount; i++) {
            worker[i] = std::thread(workerTest, image, i, resX, resY, cameraPos, cameraDir, tree, triangles,
                                    threadcount);
        }

        for (int i = 0; i < threadcount; i++) {
            worker[i].join();
        }

        /*for (int i = 0; i < resX * resY; i++) {
            image[i] = shade(testIntersection(createRay(resX, resY, i, 1500, cameraPos, cameraDir), tree, triangles));
        }*/


        display(image, resX, resY);

        window.display();

        auto ts = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(
                ts - reference);
        /*std::this_thread::sleep_for(
                std::chrono::duration<double>((17 - ((int) (time_span.count() * 1000)) % 17)) / 1000.f);*/
        frames++;
        if (frames % 60 == 0)
            std::cout << "FPS: " << frames / time_span.count() << std::endl;
    }

    free(image);
}

#endif //HEAP_BVH_TESTRENDERER_H
