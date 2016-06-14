#pragma once

#include <armadillo>
#include <string>
#include <vector>
#include <boost/assert.hpp>
#include <boost/format.hpp>


struct ProjConst {
    static const std::string working_root;
    static const std::string project_root;
    static const std::vector<std::string> dsnames;
};

const std::string ProjConst::working_root = "C:/Users/Justin/workspace";
const std::string ProjConst::project_root = ProjConst::working_root + "/PWP3D";

const std::vector<std::string> ProjConst::dsnames =
    std::vector<std::string>{"SJ21", "SJ31", "SJ41", "SJ51", "SJ22", "SJ32",
                             "SJ23", "SJ33", "SJ53", "SJ34", "SJ54"};

struct PWP3DInput {
    std::string sModelPath;
    std::string sSrcImage;

    std::string sCameraMatrix;

    std::string sTargetMask;
    std::string sHistSrc;
    std::string sHistMask;

    std::string camrtpath;
    std::string modelrtpath;
};

struct Input {
    std::string sModelPath;
    std::string sSrcImage;
    std::string sCameraMatrix;

    std::string camrtpath;
    std::string modelrtpath;
};

template <typename InputType>
InputType capgDs(std::string dsname, int vidx){
    BOOST_ASSERT_MSG(false, "return empty input-struct");
    return InputType();
}

template <typename InputType>
InputType capgDs(std::string dsname){
    BOOST_ASSERT_MSG(false, "return empty input-struct");
    return InputType();
}


template<>
PWP3DInput capgDs<PWP3DInput>(std::string dsname, int vidx){
    /* default 0: the first frame
       -1: all frames (not supported)
    */
    PWP3DInput in;

    in.sModelPath =std::string(ProjConst::project_root) + "/Files/Models/Renderer/antenna.obj";

    in.sSrcImage = boost::str(boost::format("%s/%s/%s/pic_%d.bmp") % ProjConst::project_root % "Files/fan/Images" % dsname % vidx);
    in.sHistSrc = in.sSrcImage; //this is the same as src

    in.sCameraMatrix = boost::str(boost::format("%s/%s/%s.cal") % ProjConst::project_root % "Files/fan/cam" % dsname);

    in.sTargetMask = boost::str(boost::format("%s/%s/%s/targetmask.bmp") % ProjConst::project_root % "Files/fan/Others" % dsname); // make this all white (globally effective)

    in.sHistMask = boost::str(boost::format("%s/%s/%s/pic_%d.bmp") % ProjConst::project_root % "Files/fan/HistMask" % dsname % vidx); // make this all white (globally effective)

    in.camrtpath = boost::str(boost::format("%s/%s/%s.rt.txt") % ProjConst::project_root % "Files/fan/cam" % dsname); //v
    in.modelrtpath = boost::str(boost::format("%s/%s/%s.txt") % ProjConst::project_root % "Files/fan/part" % dsname); //v

    return in;
}


template <typename MatType> void describe(const MatType &T) {
    std::cout << T.n_rows << "  " << T.n_cols << std::endl;
}

template <typename InputType> std::vector<arma::mat> init(InputType in) {
    std::vector<arma::mat> mv_mats;
    {
        arma::mat camera_rts;
        arma::mat model_rt;
        camera_rts.load(in.camrtpath);
        model_rt.load(in.modelrtpath);

        for (int iv = 0; iv < camera_rts.n_rows; iv += 3) {
            arma::mat cam_rt (4,4, arma::fill::eye);
            cam_rt.rows(0,2) = camera_rts.rows(iv, iv + 2);

            arma::mat mv = cam_rt * model_rt;
            mv_mats.push_back(mv);
        }
    }

    return mv_mats;
}

struct CamInfo{
    double fx, fy;
    double cx, cy;
    uint w, h;
    std::string name;

    static CamInfo load(std::string filename){
        std::ifstream in(filename);
        CamInfo cinfo;
        in>>cinfo.name;
        in>>cinfo.w>>cinfo.h;
        in>>cinfo.fx>>cinfo.fy;
        in>>cinfo.cx>>cinfo.cy;
        in.close();
        return cinfo;
    }
};
