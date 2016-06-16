#include "../PerseusLib/PerseusLib.h"
#include "../PerseusLib/capgdatautils.hpp"

#include "Utils/Timer.h"
#include <opencv2/opencv.hpp>
#include <armadillo>

#include <string>
#include <vector>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

using namespace std;

using namespace Perseus::Utils;
using namespace std;

string working_root = ProjConst::working_root;
string project_root = ProjConst::project_root;
auto dsnames = ProjConst::dsnames;

using namespace boost;


int main(int argc, char *argv[])
{
	bool singleview = false;

	if (argc == 3)
		singleview = true;

	std::string dsname = argv[1];

	int selected_idx = -1;
	if (singleview){
		selected_idx = stoi(std::string(argv[2]));
	}

	auto tins = capgDs<PWP3DInput>(dsname, 0);

	auto mv_mats = init(tins);

	char str_result[100];
	int i;

	// this is from image size
	int width = 1920, height = 1080;

	// fix view count
	int viewCount = mv_mats.size();
	if (singleview)
		viewCount = 1;

	int objectCount = 1;

	int objectId = 0;
	int objectIdx = 0;

	Timer t;

	//result visualisation
	ImageUChar4* ResultImage = new ImageUChar4(width, height);

  //objects allocation + initialisation: 3d model in obj required
  Object3D **objects = new Object3D*[objectCount];

	cout << "\n==[APP] Init Model ==" << endl;
	objects[objectIdx] = new Object3D(objectId, viewCount, (char*)tins.sModelPath.c_str(), width, height);

	View3D **views = new View3D*[viewCount];
	IterationConfiguration *iterConfig = new IterationConfiguration();

	iterConfig->width = width; iterConfig->height = height;
	iterConfig->levelSetBandSize = 20;
	iterConfig->iterViewCount = viewCount; //to change

	iterConfig->iterCount = 50;
	iterConfig->useCUDAEF = false;
	iterConfig->useCUDARender = false;

	float sx = 1000.f;
	float sy = 10000.f;
	float sz = 1000.f;
	float sr = 0.01f;

	std::string identifier_str = boost::str(boost::format("pwp3d_%s_v_%d_b_%d_I_%d_%f_%f_%f") % dsname % selected_idx % iterConfig->levelSetBandSize % iterConfig->iterCount % sr % sx %sy);
	iterConfig->idstr = identifier_str;

	std::cout << identifier_str << std::endl;

	//primary initilisation
	OptimisationEngine::Instance()->Initialise(width, height);

	int prev_vp = -1;
	int next_vp;
	for (int viewIdx = 0; viewIdx < viewCount; ++viewIdx) {
		if (singleview){
			while (true){
				next_vp = nextValidIdx(prev_vp, dsname);
				if (next_vp == selected_idx) break;
			}
		}
		else{
			next_vp = nextValidIdx(prev_vp, dsname);
			prev_vp = next_vp;
		}

		auto ins = capgDs<PWP3DInput>(dsname, next_vp);
		std::cout << "viewpoint: " << next_vp;

		cout << "\n==[APP] Init CameraMatrix ==" << endl;
		ImageUChar4* camera = new ImageUChar4(width, height);
		ImageUtils::Instance()->LoadImageFromFile(camera, (char*)ins.sSrcImage.c_str());

		views[viewIdx] = new View3D(viewIdx, (char*)ins.sCameraMatrix.c_str(), width, height);

		cout << "\n==[APP] Init Target ROI ==" << endl;
		ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
			(char*)ins.sTargetMask.c_str());

		ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
			(char*)ins.sHistSrc.c_str());

		ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
			(char*)ins.sHistMask.c_str(), objectIdx + 1);
		// ---------------------------------------------------------------------------
		//iteration configuration for one object
		iterConfig->iterViewIds[viewIdx] = viewIdx;
		iterConfig->iterObjectCount[viewIdx] = 1; //each view contains 1 object
		iterConfig->iterObjectIds[viewIdx][objectIdx] = 0;

		//step size per object and view
		//objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 10.f, 10.f, 100.0f);
		//objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.01f, 10000.f, 10000.f, 10000.0f); // test1
		objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(sr, sx, sy, sz);

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

		//?? how to update??
		HistogramEngine::Instance()->UpdateVarBinHistogram(
			objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
			objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


		OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

		// ---------------------------------------------------------------------------
		cout << "\n==[APP] Rendering object initial pose.. ==" << endl;
		VisualisationEngine::Instance()->GetImage(
			ResultImage, GETIMAGE_FILL,
			objects[objectIdx], views[viewIdx],
			objects[objectIdx]->initialPose[viewIdx]);

		cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);

		//cv::imshow("pose with our result", ResultMat);
		//cv::waitKey(-1);

	}
	cout << "[App] Finish Rendered object initial pose." << endl;

	printf("======= mode: useCUDAAEF: %d, use CUDARender %d ========;\n", iterConfig->useCUDAEF, iterConfig->useCUDARender);

	sprintf(str_result, "C:/Users/Justin/workspace/PWP3D/Files/Results/result%04d.png", 0);

	//main processing
	t.restart();
	OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
	t.check("Iteration");

	int testviewIdx = 0;
	//result plot
	VisualisationEngine::Instance()->GetImage(
		ResultImage, GETIMAGE_PROXIMITY,
		objects[objectIdx], views[testviewIdx], objects[objectIdx]->pose[testviewIdx]);

	//result save to file
	cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);
	cv::imwrite(identifier_str + ".jpg", ResultMat);

	char large_buff[1000];
	sprintf(large_buff, "%f %f %f\n %f %f %f %f\n",
		objects[objectIdx]->pose[testviewIdx]->translation->x,
		objects[objectIdx]->pose[testviewIdx]->translation->y,
		objects[objectIdx]->pose[testviewIdx]->translation->z,
		objects[objectIdx]->pose[testviewIdx]->rotation->vector4d.x,
		objects[objectIdx]->pose[testviewIdx]->rotation->vector4d.y,
		objects[objectIdx]->pose[testviewIdx]->rotation->vector4d.z,
		objects[objectIdx]->pose[testviewIdx]->rotation->vector4d.w);

	std::ofstream out(identifier_str + ".txt");
	out << large_buff;

	//posteriors plot
	sprintf(str_result, (identifier_str + "_post.png").c_str());

	VisualisationEngine::Instance()->GetImage(
		ResultImage, GETIMAGE_POSTERIORS,
		objects[objectIdx], views[testviewIdx], objects[objectIdx]->pose[testviewIdx]);

	ImageUtils::Instance()->SaveImageToFile(ResultImage, str_result);

	//primary engine destructor
	OptimisationEngine::Instance()->Shutdown();

	for (i = 0; i < objectCount; i++) delete objects[i];
	delete[]objects;

	for (i = 0; i < viewCount; i++) delete views[i];
	delete views;

	delete ResultImage;

	cout << "Exit pwp3D app successfully." << endl;
	return 0;
}

