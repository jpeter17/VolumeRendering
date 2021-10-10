using namespace std;

#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkInteractorStyle.h"
#include "vtkObjectFactory.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCamera.h"
#include "vtkLight.h"
#include "vtkOpenGLPolyDataMapper.h"
#include "vtkJPEGReader.h"
#include "vtkImageData.h"
#include <vtkPNGWriter.h>

#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPolyDataReader.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>
#include <vtkDataSetReader.h>
#include <vtkContourFilter.h>
#include <vtkRectilinearGrid.h>

#include <vtkCamera.h>
#include <vtkDataSetMapper.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

#include <vtkMath.h>
#include <iostream>

#define PI 3.14159265
#define WIDTH 1000
#define HEIGHT 1000
#define SAMPLES 1000

struct Camera
{
    double          near, far;
    double          angle;
    double          position[3];
    double          focus[3];
    double          up[3];
};


struct TransferFunction
{
    double          min;
    double          max;
    int             numBins;
    unsigned char* colors;  // size is 3*numBins
    double* opacities; // size is numBins

    // Take in a value and applies the transfer function.
    // Step #1: figure out which bin "value" lies in.
    // If "min" is 2 and "max" is 4, and there are 10 bins, then
    //   bin 0 = 2->2.2
    //   bin 1 = 2.2->2.4
    //   bin 2 = 2.4->2.6
    //   bin 3 = 2.6->2.8
    //   bin 4 = 2.8->3.0
    //   bin 5 = 3.0->3.2
    //   bin 6 = 3.2->3.4
    //   bin 7 = 3.4->3.6
    //   bin 8 = 3.6->3.8
    //   bin 9 = 3.8->4.0
    // and, for example, a "value" of 3.15 would return the color in bin 5
    // and the opacity at "opacities[5]".
    int GetBin(double value) {
        int bin = -1;
        if (value >= min && value <= max) {
            value -= min;
            double binSize = (max - min) / numBins;
            bin = floor(value / binSize);
            
        }
        return bin;
    }

    void ApplyTransferFunction(double value, unsigned char* RGB, double& opacity)
    {
        int bin = GetBin(value);
        if (bin != -1) {
            RGB[0] = colors[3 * bin + 0];
            RGB[1] = colors[3 * bin + 1];
            RGB[2] = colors[3 * bin + 2];
            opacity = opacities[bin];
            //cout << "Mapped to bin " << bin << endl;
        }
        else {
            RGB[0] = 0;
            RGB[1] = 0;
            RGB[2] = 0;
            opacity = 0;
            //cout << "Out of range: no valid bin" << endl;
        }
    }
};

TransferFunction
SetupTransferFunction(void)
{
    int  i;

    TransferFunction rv;
    rv.min = 10;
    rv.max = 15;
    rv.numBins = 256;
    rv.colors = new unsigned char[3 * 256];
    rv.opacities = new double[256];
    unsigned char charOpacity[256] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 4, 3, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 17, 17, 17, 17, 17, 16, 16, 15, 14, 13, 12, 11, 9, 8, 7, 6, 5, 5, 4, 3, 3, 3, 4, 5, 6, 7, 8, 9, 11, 12, 14, 16, 18, 20, 22, 24, 27, 29, 32, 35, 38, 41, 44, 47, 50, 52, 55, 58, 60, 62, 64, 66, 67, 68, 69, 70, 70, 70, 69, 68, 67, 66, 64, 62, 60, 58, 55, 52, 50, 47, 44, 41, 38, 35, 32, 29, 27, 24, 22, 20, 20, 23, 28, 33, 38, 45, 51, 59, 67, 76, 85, 95, 105, 116, 127, 138, 149, 160, 170, 180, 189, 198, 205, 212, 217, 221, 223, 224, 224, 222, 219, 214, 208, 201, 193, 184, 174, 164, 153, 142, 131, 120, 109, 99, 89, 79, 70, 62, 54, 47, 40, 35, 30, 25, 21, 17, 14, 12, 10, 8, 6, 5, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    for (i = 0; i < 256; i++)
        rv.opacities[i] = charOpacity[i] / 255.0;
    const int numControlPoints = 8;
    unsigned char controlPointColors[numControlPoints * 3] = {
           71, 71, 219, 0, 0, 91, 0, 255, 255, 0, 127, 0,
           255, 255, 0, 255, 96, 0, 107, 0, 0, 224, 76, 76
    };
    double controlPointPositions[numControlPoints] = { 0, 0.143, 0.285, 0.429, 0.571, 0.714, 0.857, 1.0 };
    for (i = 0; i < numControlPoints - 1; i++)
    {
        int start = controlPointPositions[i] * rv.numBins;
        int end = controlPointPositions[i + 1] * rv.numBins + 1;
        cerr << "Working on " << i << "/" << i + 1 << ", with range " << start << "/" << end << endl;
        if (end >= rv.numBins)
            end = rv.numBins - 1;
        for (int j = start; j <= end; j++)
        {
            double proportion = (j / (rv.numBins - 1.0) - controlPointPositions[i]) / (controlPointPositions[i + 1] - controlPointPositions[i]);
            if (proportion < 0 || proportion > 1.)
                continue;
            for (int k = 0; k < 3; k++)
                rv.colors[3 * j + k] = proportion * (controlPointColors[3 * (i + 1) + k] - controlPointColors[3 * i + k])
                + controlPointColors[3 * i + k];
        }
    }

    return rv;
}

Camera
SetupCamera(void)
{
    Camera rv;
    rv.focus[0] = 0;
    rv.focus[1] = 0;
    rv.focus[2] = 0;
    rv.up[0] = 0;
    rv.up[1] = -1;
    rv.up[2] = 0;
    rv.angle = 30;
    rv.near = 7.5e+7;
    rv.far = 1.4e+8;
    rv.position[0] = -8.25e+7;
    rv.position[1] = -3.45e+7;
    rv.position[2] = 3.35e+7;

    return rv;
}

void
WriteImage(vtkImageData* img, const char* filename)
{
    std::string full_filename = filename;
    full_filename += ".png";
    vtkPNGWriter* writer = vtkPNGWriter::New();
    writer->SetInputData(img);
    writer->SetFileName(full_filename.c_str());
    writer->Write();
    writer->Delete();
}

vtkImageData*
NewImage(int width, int height)
{
    vtkImageData* image = vtkImageData::New();
    image->SetDimensions(width, height, 1);
    //image->SetWholeExtent(0, width-1, 0, height-1, 0, 0);
    //image->SetUpdateExtent(0, width-1, 0, height-1, 0, 0);
    //image->SetNumberOfScalarComponents(3);
    image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    //image->AllocateScalars();

    return image;
}

void rd(int i, int j, double look[3], double R_x[3], double R_y[3], double R_d[3]) {
    double norm = vtkMath::Norm(look);
    double xScalar = 2 * i;
    xScalar += 1; 
    xScalar -= WIDTH;
    xScalar /= 2;
    double yScalar = 2 * j;
    yScalar += 1;
    yScalar -= HEIGHT;
    yScalar /= 2;
    for (int i = 0; i < 3; i++) {
        R_d[i] = look[i] * (1 / norm) + xScalar * R_x[i] + yScalar * R_y[i];
    }
}

// ****************************************************************************
//  Function: GetPointIndex
//
//  Arguments:
//      idx:  the logical index of a point.
//              0 <= idx[0] < dims[0]
//              1 <= idx[1] < dims[1]
//              2 <= idx[2] < dims[2] (or always 0 if 2D)
//      dims: an array of size 3 with the number of points in X, Y, and Z.
//            2D data sets would have Z=1
//
//  Returns:  the point index
//
// ****************************************************************************

int GetPointIndex(const int* idx, const int* dims)
{
    // 3D
    return idx[2]*dims[0]*dims[1]+idx[1]*dims[0]+idx[0];
    // 2D
    //return idx[1] * dims[0] + idx[0];
}

// ****************************************************************************
//  Function: BinarySearch
//
//  Arguments:
//     value: a location
//     X: an array (size is specified by dim).  
//              This contains locations of a rectilinear mesh.
//     dims: index of X
//
//   Returns: index of location nearest value without going over
//
// ****************************************************************************

int BinarySearch(const float value, const float* X, int dim) {
    if (value < X[0]) {
        return 0;
    }
    if (value > X[dim - 1]) {
        return dim - 1;
    }

    int lo = 0;
    int hi = dim - 1;

    while (lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (value >= X[mid]) {
            lo = mid;
        }
        else if (value < X[mid]) {
            hi = mid;
        }
    }

    return lo;
}

// ****************************************************************************
//  Function: EvaluateVectorFieldAtLocation
//
//  Arguments:
//     pt: a two-dimensional location
//     dims: an array of size two.  
//              The first number is the size of the array in argument X, 
//              the second the size of Y.
//     X: an array (size is specified by dims).  
//              This contains the X locations of a rectilinear mesh.
//     Y: an array (size is specified by dims).  
//              This contains the Y locations of a rectilinear mesh.
//     F: a vector field defined on the mesh.  Its size is 2*dims[0]*dims[1].
//        The first value in the field is the x-component for the first point.
//        The second value in the field is the y-component for the first point.
//
//     rv (output): the interpolated field value. (0,0) if the location is out of bounds.
//
// ****************************************************************************

void EvaluateVectorFieldAtLocation(float* pt, const int* dims, const float* X,
    const float* Y, const float* Z, const float* F)
{
    int idx[3], ptIdx;
    float xOffset, yOffset, zOffset;
    float frontUp, frontDown, backUp, backDown;
    float front, back;
    float val = 0; 

    if (pt[0] >= X[0] && pt[0] <= X[dims[0] - 1] &&
        pt[1] >= Y[0] && pt[1] <= Y[dims[1] - 1] &&
        pt[2] >= Z[0] && pt[2] <= Z[dims[2] - 1]) 
    {
        idx[0] = BinarySearch(pt[0], X, dims[0]);
        idx[1] = BinarySearch(pt[1], Y, dims[1]);
        idx[2] = BinarySearch(pt[2], Z, dims[2]);

        xOffset = (pt[0] - X[idx[0]]) / (X[idx[0] + 1] - X[idx[0]]);
        yOffset = (pt[1] - Y[idx[1]]) / (Y[idx[1] + 1] - Y[idx[1]]);
        zOffset = (pt[2] - Z[idx[2]]) / (Z[idx[2] + 1] - Z[idx[2]]);

        ptIdx = GetPointIndex(idx, dims);

        frontDown = F[ptIdx] + xOffset * (F[ptIdx + 1] - F[ptIdx]);
        frontUp = F[ptIdx + dims[0]] + xOffset * (F[ptIdx + 1 + dims[0]] - F[ptIdx + dims[0]]);
        backDown = F[ptIdx + (dims[0] * dims[1])] + xOffset * (F[ptIdx + 1 + (dims[0] * dims[1])] - F[ptIdx + (dims[0] * dims[1])]);
        backUp = F[ptIdx + dims[0] + (dims[0] * dims[1])] + xOffset * (F[ptIdx + 1 + dims[0] + (dims[0] * dims[1])] - F[ptIdx + dims[0] + (dims[0] * dims[1])]);
        front = frontDown + yOffset * (frontUp - frontDown);
        back = backDown + yOffset * (backUp - backDown);
        val = front + zOffset * (back - front);
    }

    pt[3] = val;
}

void FindPixelColor(int x, int y, Camera cam, TransferFunction tf, double stepSize, double look[3], double R_x[3], double R_y[3], const int* dims,
    const float* X, const float* Y, const float* Z, const float* F, unsigned char RGB[3]) 
{
    double R_d[3];
    rd(x, y, look, R_x, R_y, R_d);

    float sample[SAMPLES][4];
    for (int i = 0; i < SAMPLES; i++) {
        double dist = cam.near + i * stepSize;
        for (int j = 0; j < 3; j++) {
            sample[i][j] = cam.position[j] + R_d[j] * dist;
        }
        EvaluateVectorFieldAtLocation(sample[i], dims, X, Y, Z, F);
    }
    double r = 0;
    double g = 0;
    double b = 0;
    double a = 0;
    for (int i = 0; i < SAMPLES; i++) {
        unsigned char RGB[3];
        double opacity;
        tf.ApplyTransferFunction(sample[i][3], RGB, opacity);
        opacity = 1 - pow(1 - opacity, 500 / (double)SAMPLES);
        r = r + (1 - a) * opacity * ((double)RGB[0] / 255);
        g = g + (1 - a) * opacity * ((double)RGB[1] / 255);
        b = b + (1 - a) * opacity * ((double)RGB[2] / 255);
        a = a + (1 - a) * opacity;
    }
    RGB[0] = floor(r * 255);
    RGB[1] = floor(g * 255);
    RGB[2] = floor(b * 255);
}

int main(int argc, char* argv[])
{
    TransferFunction tf = SetupTransferFunction();
    for (int i = 0; i < tf.numBins; i++)
    {
        cerr << i << ": " << (int)tf.colors[3 * i] << ", " << (int)tf.colors[3 * i + 1] << ", " << (int)tf.colors[3 * i + 2] << ", " << tf.opacities[i] << endl;
    }

    vtkDataSetReader* rdr = vtkDataSetReader::New();
    rdr->SetFileName("astro512.vtk");
    rdr->Update();

    int dims[3];
    vtkRectilinearGrid* rgrid = (vtkRectilinearGrid*)rdr->GetOutput();
    rgrid->GetDimensions(dims);

    float* X = (float*)rgrid->GetXCoordinates()->GetVoidPointer(0);
    float* Y = (float*)rgrid->GetYCoordinates()->GetVoidPointer(0);
    float* Z = (float*)rgrid->GetZCoordinates()->GetVoidPointer(0);
    float* F = (float*)rgrid->GetPointData()->GetScalars()->GetVoidPointer(0);
    
    vtkImageData* image = NewImage(WIDTH, HEIGHT);
    unsigned char* buffer = (unsigned char*)image->GetScalarPointer(0, 0, 0);

    Camera cam = SetupCamera();

    double look[3];
    vtkMath::Subtract(cam.focus, cam.position, look);
    double R_u[3];
    double R_v[3];

    vtkMath::Cross(look, cam.up, R_u);
    vtkMath::MultiplyScalar(R_u, 1 / vtkMath::Norm(R_u));

    vtkMath::Cross(look, R_u, R_v);
    vtkMath::MultiplyScalar(R_v, 1 / vtkMath::Norm(R_v));

    double xScalar = (2 * tan((cam.angle * PI) / (2 * 180))) / WIDTH;
    double yScalar = (2 * tan((cam.angle * PI) / (2 * 180))) / HEIGHT;

    double R_x[3];
    double R_y[3];
    for (int i = 0; i < 3; i++) {
        R_x[i] = R_u[i] * xScalar;
        R_y[i] = R_v[i] * yScalar;
    }

    

    double dist = cam.far - cam.near;
    const int samples = SAMPLES;
    double stepSize = dist / (samples - 1);
    
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            int offset = 3 * (i * WIDTH + j);
            FindPixelColor(j, i, cam, tf, stepSize, look, R_x, R_y, dims, X, Y, Z, F, buffer+offset);
        }
    }

    WriteImage(image, "VolumeRendering");
}