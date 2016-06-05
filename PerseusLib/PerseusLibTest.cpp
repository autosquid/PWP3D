#include "../PerseusLib/PerseusLib.h"

#include "Utils/Timer.h"
#include <opencv2/opencv.hpp>
#include <armadillo>

#include <string>
#include <vector>

#include <boost/format.hpp>

using namespace std;

using namespace Perseus::Utils;
using namespace std;

// this is the const for all my proects
string working_root = "C:/Users/Justin/workspace";

// this project
string project_root = working_root + "/PWP3D";


auto dsnames = vector<string>{"SJ21", "SJ31", "SJ41", "SJ51", "SJ22", "SJ32", "SJ52", "SJ23", "SJ33", "SJ53", "SJ34", "SJ54"};

string dsname = dsnames[0];

struct Input{
	string sModelPath;

	string sSrcImage;
	//string sSrcImage = project_root + "/Files/Images/Red.png"; //

	string sCameraMatrix; //v
	//string sCameraMatrix = project_root +  "/Files/CameraCalibration/900nc.cal"; //v

	string sTargetMask;
	//string sTargetMask = project_root + "/Files/Masks/480p_All_VideoMask.png";

	string sHistSrc;
	//string sHistSrc = project_root + "/Files/Masks/Red_Source.png";

	//string sHistMask = project_root + "/Files/Masks/Red_Mask.png";
	string sHistMask;

	string camrtpath; //v
	string modelrtpath; //v
};



using namespace boost;

Input capgDs(std::string dsname, int vidx = 0){
	/* default 0: the first frame
	-1: all frames (not supported)
	*/
	Input in;

	in.sModelPath = project_root + "/Files/Models/Renderer/antenna.obj";

	in.sSrcImage = str(format("%s/%s/%s/pic_%d.bmp") % project_root % "Files/fan/Images" % dsname % vidx);
	in.sHistSrc = in.sSrcImage; //this is the same as src

	in.sCameraMatrix = str(format("%s/%s/%s.cal") % project_root % "Files/fan/cam" % dsname);

	in.sTargetMask = str(format("%s/%s/%s/targetmask.bmp") % project_root % "Files/fan/Others" % dsname); // make this all white (globally effective)

	in.sHistMask = str(format("%s/%s/%s/pic_%d.bmp") % project_root % "Files/fan/HistMask" % dsname % vidx); // make this all white (globally effective)

	in.camrtpath = str(format("%s/%s.rt.txt") % (project_root + "/Files/fan/cam") % dsname); //v
	in.modelrtpath = str(format("%s/%s.txt") % (project_root + "/Files/fan/rt") % dsname); //v

	return in;
}


template <typename MatType>
void describe(const MatType& T){
	cout << T.n_rows << "  " << T.n_cols << endl;
}


std::vector<arma::mat> init(Input in) {

	vector<arma::mat> mv_mats;
	{//load model-view r/t
		arma::mat camera_rts;
		arma::mat model_rt;
		camera_rts.load(in.camrtpath);
		model_rt.load(in.modelrtpath);
		cout << camera_rts.size() << endl << camera_rts << endl;
		describe(camera_rts);

		cout << model_rt.size() << endl << model_rt << endl;
		describe(model_rt);

		for (int iv = 0; iv < camera_rts.n_rows; iv += 4){
			arma::mat mv = camera_rts.rows(iv, iv + 3) * model_rt;
			mv_mats.push_back(mv);
		}
	}

	for (auto m : mv_mats){
		cout << m << endl;
	}

	return mv_mats;
}


int main(void)
{
	auto ins = capgDs("SJ21");
	auto mv_mats = init(ins);

	char str_result[100];
	int i;

	// this is from image size
	int width = 1920, height = 1080;

	// fix view count
	int viewCount = 3;

	int objectCount = 1;

	int objectId = 0;
	int objectIdx = 0;


	Timer t;

	//result visualisation
	ImageUChar4* ResultImage = new ImageUChar4(width, height);

	//objects allocation + initialisation: 3d model in obj required
	Object3D **objects = new Object3D*[objectCount];

	cout << "\n==[APP] Init Model ==" << endl;
	objects[objectIdx] = new Object3D(objectId, viewCount, (char*)ins.sModelPath.c_str(), width, height);

	View3D **views = new View3D*[viewCount];
	IterationConfiguration *iterConfig = new IterationConfiguration();


	//primary initilisation
	OptimisationEngine::Instance()->Initialise(width, height);

	for (int viewIdx = 0; viewIdx < viewCount; ++viewIdx)
	{
		auto ins = capgDs("SJ21", viewIdx);

		cout << "\n==[APP] Init CameraMatrix ==" << endl;
		ImageUChar4* camera = new ImageUChar4(width, height);
		ImageUtils::Instance()->LoadImageFromFile(camera, (char*)ins.sSrcImage.c_str());

		views[viewIdx] = new View3D(0, (char*)ins.sCameraMatrix.c_str(), width, height);

		cout << "\n==[APP] Init Target ROI ==" << endl;
		ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
			(char*)ins.sTargetMask.c_str());

		ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
			(char*)ins.sHistSrc.c_str());

		ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
			(char*)ins.sHistMask.c_str(), objectIdx + 1);

		//?? how to update??
		HistogramEngine::Instance()->UpdateVarBinHistogram(
			objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
			objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


		// ---------------------------------------------------------------------------
		//iteration configuration for one object
		iterConfig->width = width; iterConfig->height = height;
		iterConfig->iterViewIds[viewIdx] = 0; //?? how to set this??
		iterConfig->iterObjectCount[viewIdx] = 1;
		iterConfig->levelSetBandSize = 8;
		iterConfig->iterObjectIds[viewIdx][objectIdx] = 0; //???
		iterConfig->iterViewCount = 1; //to change
		iterConfig->iterCount = 1;

		//step size per object and view
		objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 0.5f, 0.5f, 10.0f);


		float mv_data[9];
		for (int ii = 0; ii < 3; ++ii){
			for (int jj = 0; jj < 3; ++jj) mv_data[ii * 3 + jj] = mv_mats[viewIdx].at(ii, jj);
		}
		Quaternion q;
		q.FromMatrix(mv_data);

		objects[objectIdx]->initialPose[viewIdx]->SetFrom(
			mv_mats[viewIdx].at(0, 3),
			mv_mats[viewIdx].at(1, 3),
			mv_mats[viewIdx].at(2, 3),
			q.vector4d.x,
			q.vector4d.y,
			q.vector4d.z,
			q.vector4d.w);

		//register camera image with main engine
		// todo ??? how many ??
		OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

		// ---------------------------------------------------------------------------
		cout << "\n==[APP] Rendering object initial pose.. ==" << endl;
		VisualisationEngine::Instance()->GetImage(
			ResultImage, GETIMAGE_PROXIMITY,
			objects[objectIdx], views[viewIdx],
			objects[objectIdx]->initialPose[viewIdx]);

		cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);

		cv::imshow("pose with our result", ResultMat);

		cout << "[cam info]: ";
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				cout << views[viewIdx]->renderView->camera3D->K[i][j] << ' ';
		cout << endl;


		cv::waitKey(2000);

		cout << "[App] Finish Rendered object initial pose." << endl;
	}

	int viewIdx = 0;
	for (i = 0; i < 4; i++)
	{
		switch (i)
		{
		case 0:
			iterConfig->useCUDAEF = true;
			iterConfig->useCUDARender = true;
			break;
		case 1:
			iterConfig->useCUDAEF = false;
			iterConfig->useCUDARender = true;
			break;
		case 2:
			iterConfig->useCUDAEF = true;
			iterConfig->useCUDARender = false;
			break;
		case 3:
			iterConfig->useCUDAEF = false;
			iterConfig->useCUDARender = false;
			break;
		}

		printf("======= mode: useCUDAAEF: %d, use CUDARender %d ========;\n", iterConfig->useCUDAEF, iterConfig->useCUDARender);

		sprintf(str_result, "C:/Users/Justin/workspace/PWP3D/Files/Results/result%04d.png", i);

		//main processing
		t.restart();
		OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
		t.check("Iteration");

		//result plot
		VisualisationEngine::Instance()->GetImage(
			ResultImage, GETIMAGE_PROXIMITY,
			objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

		//result save to file
		cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);
		cv::imshow("result", ResultMat);
		cv::waitKey(2000);
		cv::imwrite(str_result, ResultMat);

		printf("final pose result %f %f %f %f %f %f %f\n\n",
			objects[objectIdx]->pose[viewIdx]->translation->x,
			objects[objectIdx]->pose[viewIdx]->translation->y,
			objects[objectIdx]->pose[viewIdx]->translation->z,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);

		break; // one is enough
	}

	//posteriors plot
	sprintf(str_result, "C:/Users/Justin/workspace/PWP3D/Files/Results/posteriors.png");
	VisualisationEngine::Instance()->GetImage(
		ResultImage, GETIMAGE_POSTERIORS,
		objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

	ImageUtils::Instance()->SaveImageToFile(ResultImage, str_result);

	//primary engine destructor
	OptimisationEngine::Instance()->Shutdown();

	for (i = 0; i < objectCount; i++) delete objects[i];
	delete objects;

	for (i = 0; i < viewCount; i++) delete views[i];
	delete views;

	delete ResultImage;

	cout << "Exit pwp3D app successfully." << endl;

	return 0;
}
#include "../PerseusLib/PerseusLib.h"

#include "Utils/Timer.h"
#include <opencv2/opencv.hpp>
#include <armadillo>

#include <string>
#include <vector>

#include <boost/format.hpp>

using namespace std;

using namespace Perseus::Utils;
using namespace std;

// this is the const for all my proects
string working_root = "C:/Users/Justin/workspace";

// this project
string project_root = working_root + "/PWP3D";


auto dsnames = vector<string>{"SJ21", "SJ31", "SJ41", "SJ51", "SJ22", "SJ32", "SJ52", "SJ23", "SJ33", "SJ53", "SJ34", "SJ54"};

string dsname = dsnames[0];

struct Input{
	string sModelPath;

	string sSrcImage;
	//string sSrcImage = project_root + "/Files/Images/Red.png"; //

	string sCameraMatrix; //v
	//string sCameraMatrix = project_root +  "/Files/CameraCalibration/900nc.cal"; //v

	string sTargetMask;
	//string sTargetMask = project_root + "/Files/Masks/480p_All_VideoMask.png";

	string sHistSrc;
	//string sHistSrc = project_root + "/Files/Masks/Red_Source.png";

	//string sHistMask = project_root + "/Files/Masks/Red_Mask.png";
	string sHistMask;

	string camrtpath; //v
	string modelrtpath; //v
};



using namespace boost;

Input capgDs(std::string dsname, int vidx = 0){
	/* default 0: the first frame
	-1: all frames (not supported)
	*/
	Input in;

	in.sModelPath = project_root + "/Files/Models/Renderer/antenna.obj";

	in.sSrcImage = str(format("%s/%s/%s/pic_%d.bmp") % project_root % "Files/fan/Images" % dsname % vidx);
	in.sHistSrc = in.sSrcImage; //this is the same as src

	in.sCameraMatrix = str(format("%s/%s/%s.cal") % project_root % "Files/fan/cam" % dsname);

	in.sTargetMask = str(format("%s/%s/%s/targetmask.bmp") % project_root % "Files/fan/Others" % dsname); // make this all white (globally effective)

	in.sHistMask = str(format("%s/%s/%s/pic_%d.bmp") % project_root % "Files/fan/HistMask" % dsname % vidx); // make this all white (globally effective)

	in.camrtpath = str(format("%s/%s.rt.txt") % (project_root + "/Files/fan/cam") % dsname); //v
	in.modelrtpath = str(format("%s/%s.txt") % (project_root + "/Files/fan/rt") % dsname); //v

	return in;
}


template <typename MatType>
void describe(const MatType& T){
	cout << T.n_rows << "  " << T.n_cols << endl;
}


std::vector<arma::mat> init(Input in) {

	vector<arma::mat> mv_mats;
	{//load model-view r/t
		arma::mat camera_rts;
		arma::mat model_rt;
		camera_rts.load(in.camrtpath);
		model_rt.load(in.modelrtpath);
		cout << camera_rts.size() << endl << camera_rts << endl;
		describe(camera_rts);

		cout << model_rt.size() << endl << model_rt << endl;
		describe(model_rt);

		for (int iv = 0; iv < camera_rts.n_rows; iv += 4){
			arma::mat mv = camera_rts.rows(iv, iv + 3) * model_rt;
			mv_mats.push_back(mv);
		}
	}

	for (auto m : mv_mats){
		cout << m << endl;
	}

	return mv_mats;
}


int main(void)
{
	auto ins = capgDs("SJ21");
	auto mv_mats = init(ins);

	char str_result[100];
	int i;

	// this is from image size
	int width = 1920, height = 1080;

	// fix view count
	int viewCount = 3;

	int objectCount = 1;

	int objectId = 0;
	int objectIdx = 0;


	Timer t;

	//result visualisation
	ImageUChar4* ResultImage = new ImageUChar4(width, height);

	//objects allocation + initialisation: 3d model in obj required
	Object3D **objects = new Object3D*[objectCount];

	cout << "\n==[APP] Init Model ==" << endl;
	objects[objectIdx] = new Object3D(objectId, viewCount, (char*)ins.sModelPath.c_str(), width, height);

	View3D **views = new View3D*[viewCount];
	IterationConfiguration *iterConfig = new IterationConfiguration();


	//primary initilisation
	OptimisationEngine::Instance()->Initialise(width, height);

	for (int viewIdx = 0; viewIdx < viewCount; ++viewIdx)
	{
		auto ins = capgDs("SJ21", viewIdx);

		cout << "\n==[APP] Init CameraMatrix ==" << endl;
		ImageUChar4* camera = new ImageUChar4(width, height);
		ImageUtils::Instance()->LoadImageFromFile(camera, (char*)ins.sSrcImage.c_str());

		views[viewIdx] = new View3D(0, (char*)ins.sCameraMatrix.c_str(), width, height);

		cout << "\n==[APP] Init Target ROI ==" << endl;
		ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
			(char*)ins.sTargetMask.c_str());

		ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
			(char*)ins.sHistSrc.c_str());

		ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
			(char*)ins.sHistMask.c_str(), objectIdx + 1);

		//?? how to update??
		HistogramEngine::Instance()->UpdateVarBinHistogram(
			objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
			objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


		// ---------------------------------------------------------------------------
		//iteration configuration for one object
		iterConfig->width = width; iterConfig->height = height;
		iterConfig->iterViewIds[viewIdx] = 0; //?? how to set this??
		iterConfig->iterObjectCount[viewIdx] = 1;
		iterConfig->levelSetBandSize = 8;
		iterConfig->iterObjectIds[viewIdx][objectIdx] = 0; //???
		iterConfig->iterViewCount = 1; //to change
		iterConfig->iterCount = 1;

		//step size per object and view
		objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 0.5f, 0.5f, 10.0f);


		float mv_data[9];
		for (int ii = 0; ii < 3; ++ii){
			for (int jj = 0; jj < 3; ++jj) mv_data[ii * 3 + jj] = mv_mats[viewIdx].at(ii, jj);
		}
		Quaternion q;
		q.FromMatrix(mv_data);

		objects[objectIdx]->initialPose[viewIdx]->SetFrom(
			mv_mats[viewIdx].at(0, 3),
			mv_mats[viewIdx].at(1, 3),
			mv_mats[viewIdx].at(2, 3),
			q.vector4d.x,
			q.vector4d.y,
			q.vector4d.z,
			q.vector4d.w);

		//register camera image with main engine
		// todo ??? how many ??
		OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

		// ---------------------------------------------------------------------------
		cout << "\n==[APP] Rendering object initial pose.. ==" << endl;
		VisualisationEngine::Instance()->GetImage(
			ResultImage, GETIMAGE_PROXIMITY,
			objects[objectIdx], views[viewIdx],
			objects[objectIdx]->initialPose[viewIdx]);

		cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);

		cv::imshow("pose with our result", ResultMat);

		cout << "[cam info]: ";
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				cout << views[viewIdx]->renderView->camera3D->K[i][j] << ' ';
		cout << endl;


		cv::waitKey(2000);

		cout << "[App] Finish Rendered object initial pose." << endl;
	}

	int viewIdx = 0;
	for (i = 0; i < 4; i++)
	{
		switch (i)
		{
		case 0:
			iterConfig->useCUDAEF = true;
			iterConfig->useCUDARender = true;
			break;
		case 1:
			iterConfig->useCUDAEF = false;
			iterConfig->useCUDARender = true;
			break;
		case 2:
			iterConfig->useCUDAEF = true;
			iterConfig->useCUDARender = false;
			break;
		case 3:
			iterConfig->useCUDAEF = false;
			iterConfig->useCUDARender = false;
			break;
		}

		printf("======= mode: useCUDAAEF: %d, use CUDARender %d ========;\n", iterConfig->useCUDAEF, iterConfig->useCUDARender);

		sprintf(str_result, "C:/Users/Justin/workspace/PWP3D/Files/Results/result%04d.png", i);

		//main processing
		t.restart();
		OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
		t.check("Iteration");

		//result plot
		VisualisationEngine::Instance()->GetImage(
			ResultImage, GETIMAGE_PROXIMITY,
			objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

		//result save to file
		cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);
		cv::imshow("result", ResultMat);
		cv::waitKey(2000);
		cv::imwrite(str_result, ResultMat);

		printf("final pose result %f %f %f %f %f %f %f\n\n",
			objects[objectIdx]->pose[viewIdx]->translation->x,
			objects[objectIdx]->pose[viewIdx]->translation->y,
			objects[objectIdx]->pose[viewIdx]->translation->z,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
			objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);

		break; // one is enough
	}

	//posteriors plot
	sprintf(str_result, "C:/Users/Justin/workspace/PWP3D/Files/Results/posteriors.png");
	VisualisationEngine::Instance()->GetImage(
		ResultImage, GETIMAGE_POSTERIORS,
		objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

	ImageUtils::Instance()->SaveImageToFile(ResultImage, str_result);

	//primary engine destructor
	OptimisationEngine::Instance()->Shutdown();

	for (i = 0; i < objectCount; i++) delete objects[i];
	delete objects;

	for (i = 0; i < viewCount; i++) delete views[i];
	delete views;

	delete ResultImage;

	cout << "Exit pwp3D app successfully." << endl;

	return 0;
}
