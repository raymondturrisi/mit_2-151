#include "../lib/eigen/Eigen/Core"
#include "../lib/eigen/Eigen/Dense"
#include "../lib/eigen/Eigen/LU"
#include "../lib/eigen/Eigen/Eigenvalues"
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>
using namespace Eigen;
using namespace std;

typedef MatrixXd matrix_e;
typedef VectorXd vector_e;

class ss_ctrl {
    public:

        matrix_e A;
        matrix_e A_cl;
        vector_e B;
        vector_e B_ref;
        vector_e C;
        vector_e K;
        vector_e last_state;
        vector_e latest_predicted_state;
        IOFormat OctaveFmt;

        ss_ctrl(matrix_e _A, vector_e _B, vector_e _C, vector_e _K) {
            OctaveFmt = IOFormat(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
            A = _A;
            B = _B;
            C = _C;
            K = _K;
            A_cl = A-B*K.transpose();
            if(abs(A_cl.determinant()) < 0.001) {
                fprintf(stderr, "Error: A_cl is uninvertable\n");
                cout << A_cl.format(OctaveFmt) << endl;
                exit(1);
            }
            if(abs((-C.transpose()*A_cl.inverse()*B).determinant()) < 0.0001) {
                fprintf(stderr, "Error: Cannot get a B_ref, is uninvertable. See code or check elsewhere.\n");
                exit(1);
            }
            B_ref = B*(-C.transpose()*A_cl.inverse()*B).inverse();
            
            return;
        }   

        double forward_siso_ctrl(vector_e states_n, double yss) {
            last_state = states_n;
            vector_e states_n1 = A_cl*states_n+B_ref*yss;
            latest_predicted_state = states_n1;
            double output = C.transpose()*latest_predicted_state;
            return output;
        }
            
        string repr() {
            matrix_e characteristic_matrix(A.rows()+C.rows(), A.cols()+B.cols());        
            stringstream reprss;
            reprss << "A_cl = ...\n" << A_cl.format(OctaveFmt) << "\nB_ref = ...\n" << B_ref.format(OctaveFmt) << "\nC = ...\n" << C.transpose().format(OctaveFmt) << endl;
            string repr = reprss.str();
            return repr;
        }

};

