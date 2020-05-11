/* License Information
 *
 *  Copyright (C) Alexandre Boulch and Renaud Marlet
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this
 *  software and associated documentation files (the "Software"), to deal in the Software
 *  without restriction, including without limitation the rights to use, copy, modify, merge,
 *  publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
 *  to whom the Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all copies or
 *  substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 *  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 *  OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *  Note that this library relies on external libraries subject to their own license.
 *  To use this software, you are subject to the dependencies license, these licenses
 *  applies to the dependency ONLY  and NOT this code.
 *  Please refer below to the web sites for license informations:
 *       PCL, BOOST,NANOFLANN, EIGEN
 *
 * When using the software please aknowledge the  corresponding publication:
 * "Deep Learning for Robust Normal Estimation in Unstructured Point Clouds "
 * by Alexandre Boulch and Renaud Marlet
 * Symposium of Geometry Processing 2016, Computer Graphics Forum
 */

#include "Normals.h"
#include "pc_io.h"

#include <boost/program_options.hpp>


#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <time.h>
#include <cstdlib>
#include <ctime>

using namespace std;

int main(int argc, char** argv){
    srand (time(NULL));
    try{

        string input = "-1";
        string output = "out.xyz";
        int K = 100;
        int T = 1000;
        int n_phi=15;
        int n_rot=5;
        bool ua = false;
        float tol_angle_rad=0.79;
        int k_density = 5;
        string model = "";

        int c;

        opterr = 0;
        while ((c = getopt (argc, argv, "i:o:k:t:d:p:r:a:e:")) != -1)
        switch (c){
            case 'i':{
                input = optarg;
                break;
            }
            case 'o':{
                output = optarg;
                break;
            }
            case 'k':{
                stringstream sstr("");
                sstr << optarg;
                sstr >> K;
                break;
            }
            case 't':{
                stringstream sstr("");
                sstr << optarg;
                sstr >> T;
                break;
            }
            case 'd':{
                stringstream sstr("");
                sstr << optarg;
                sstr >> ua;
                break;
            }
            case 'p':{
                stringstream sstr("");
                sstr << optarg;
                sstr >> n_phi;
                break;
            }
            case 'r':{
                stringstream sstr("");
                sstr << optarg;
                sstr >> n_rot;
                break;
            }
            case 'a':{
                stringstream sstr("");
                sstr << optarg;
                sstr >> tol_angle_rad;
                break;
            }
            case 'e':{
                stringstream sstr("");
                sstr << optarg;
                sstr >> k_density;
                break;
            }
            default:{
                cout << "Unknown option character" << endl;
                return 1;
                break;
            }
            }

        if(input=="-1"){
            cout << "Error need input file" << endl;
            return 1;
        }

        // load the point cloud
        Eigen::MatrixX3d pc, normals;
        pc_load(input,pc);

        cout << "Create estimator" << endl;
        Eigen_Normal_Estimator ne(pc,normals);
        ne.get_K()=K;
        ne.get_T()=T;
        ne.density_sensitive()=ua;
    	ne.get_n_phi() = n_phi;
    	ne.get_n_rot() = n_rot;
    	ne.get_tol_angle_rad() = tol_angle_rad;
    	ne.get_K_density()= k_density;

        cout << "Estimate" << endl;
        ne.estimate_normals();

        cout << "Save" << endl;
        // save the point cloud
        pc_save(output,pc, normals);

    }catch(std::exception& e){
        std::cerr << "Unhandled Exception reached the top of main: "
                << e.what() << ", application will now exit " << std::endl;
        return 1;
    }
    return 0;
}
