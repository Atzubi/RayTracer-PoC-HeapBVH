#include "TestRenderer.h"
#include "OBJ_Loader.h"
#include "OpenCL_Kernel.h"
#include <fstream>

using namespace objl;

int main() {
/*
    //get all platforms (drivers)
    std::vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);
    if (all_platforms.size() == 0) {
        std::cout << " No platforms found. Check OpenCL installation!\n";
        exit(1);
    }
    cl::Platform default_platform = all_platforms[0];
    std::cout << "Using platform: " << default_platform.getInfo<CL_PLATFORM_NAME>() << "\n";

    //get default device of the default platform
    std::vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    if (all_devices.size() == 0) {
        std::cout << " No devices found. Check OpenCL installation!\n";
        exit(1);
    }
    cl::Device default_device = all_devices[0];
    std::cout << "Using device: " << default_device.getInfo<CL_DEVICE_NAME>() << "\n";


    cl::Context context({default_device});

    cl::Program::Sources sources;

    // kernel calculates for each element C=A+B
    std::string kernel_code =
            "           void kernel simple_add(global const int* A, global const int* B, global int* C){                        "
            "               C[get_global_id(0)]=A[get_global_id(0)]+B[get_global_id(0)];                                        "
            "           }                                                                                                       ";

    sources.push_back({kernel_code.c_str(), kernel_code.length()});

    cl::Program program(context, sources);
    if (program.build({default_device}) != CL_SUCCESS) {
        std::cout << " Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << "\n";
        exit(1);
    }


    // create buffers on the device
    cl::Buffer buffer_A(context, CL_MEM_READ_WRITE, sizeof(int) * 10);
    cl::Buffer buffer_B(context, CL_MEM_READ_WRITE, sizeof(int) * 10);
    cl::Buffer buffer_C(context, CL_MEM_READ_WRITE, sizeof(int) * 10);

    int A[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    int B[] = {0, 1, 2, 0, 1, 2, 0, 1, 2, 0};

    //create queue to which we will push commands for the device.
    cl::CommandQueue queue(context, default_device);

    //write arrays A and B to the device
    queue.enqueueWriteBuffer(buffer_A, CL_TRUE, 0, sizeof(int) * 10, A);
    queue.enqueueWriteBuffer(buffer_B, CL_TRUE, 0, sizeof(int) * 10, B);


    //run the kernel
    /*cl::KernelFunctor simple_add(cl::Kernel(program, "simple_add"), queue, cl::NullRange, cl::NDRange(10),
                                 cl::NullRange);
    simple_add(buffer_A, buffer_B, buffer_C);

    //alternative way to run the kernel
    cl::Kernel kernel_add=cl::Kernel(program,"simple_add");
    kernel_add.setArg(0,buffer_A);
    kernel_add.setArg(1,buffer_B);
    kernel_add.setArg(2,buffer_C);
    queue.enqueueNDRangeKernel(kernel_add,cl::NullRange,cl::NDRange(10),cl::NullRange);
    queue.finish();

    int C[10];
    //read result C from the device to array C
    queue.enqueueReadBuffer(buffer_C, CL_TRUE, 0, sizeof(int) * 10, C);

    std::cout << " result: \n";
    for (int i = 0; i < 10; i++) {
        std::cout << C[i] << " ";
    }*/

    //return 0;

    Loader loader;

    // Loads the input data if it exists
    if (!loader.LoadFile("../Data/mountains.obj")) return 1;

    // 3 indices define a triangle -> size = triangle count
    uint64_t size = loader.LoadedIndices.size() / 3;

    Primitive *triangles = new Primitive[size];

    uint64_t c = 0;
    uint64_t c3 = 0;

    // Puts the triangles into a more usable format
    for (auto i : loader.LoadedIndices) {
        if (c3 == 0) {
            triangles[c].v1.dim[0] = loader.LoadedVertices[i].Position.X;
            triangles[c].v1.dim[1] = loader.LoadedVertices[i].Position.Y;
            triangles[c].v1.dim[2] = loader.LoadedVertices[i].Position.Z;
        } else if (c3 == 1) {
            triangles[c].v2.dim[0] = loader.LoadedVertices[i].Position.X;
            triangles[c].v2.dim[1] = loader.LoadedVertices[i].Position.Y;
            triangles[c].v2.dim[2] = loader.LoadedVertices[i].Position.Z;
        } else {
            triangles[c].v3.dim[0] = loader.LoadedVertices[i].Position.X;
            triangles[c].v3.dim[1] = loader.LoadedVertices[i].Position.Y;
            triangles[c].v3.dim[2] = loader.LoadedVertices[i].Position.Z;
        }
        c3 = (c3 + 1) % 3;
        if (c3 == 0)
            c++;
    }

    auto start = std::chrono::high_resolution_clock::now();

    // Builds the tree
    HBVH *tree = build(triangles, size);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;

    // Prints the time needed to create the tree
    std::cout << elapsed.count() << std::endl;

    // Prints the triangle count
    std::cout << size << std::endl;

    // Writes the tree to a file
    auto myfile = std::fstream("HBVH.binary", std::ios::out | std::ios::binary);
    myfile.write((char *) &tree[0], size * 8 * 2 - 1 + 6);
    myfile.close();


    Vector cPos, cDir;
    cPos.dim[0] = 0;
    cPos.dim[1] = 0;
    cPos.dim[2] = -100;
    cDir.dim[0] = 0;
    cDir.dim[1] = 0;
    cDir.dim[2] = 1;
    render(tree, triangles, cPos, cDir);

    free(tree);
    free(triangles);

    return 0;
}