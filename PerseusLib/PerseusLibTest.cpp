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

int main(void)
{
	for (auto dsname : ProjConst::dsnames){
		auto tins = capgDs<PWP3DInput>(dsname, 0);

		auto mv_mats = init(tins);

		char str_result[100];
		int i;

		// this is from image size
		int width = 1920, height = 1080;

		// fix view count
		int viewCount = 1; mv_mats.size();

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

		std::string identifier_str = boost::str(boost::format("pwp3d_%s") % dsname);
		std::cout << identifier_str << std::endl;

		//primary initilisation
		OptimisationEngine::Instance()->Initialise(width, height);

		int next_vp=0;
		for (int viewIdx = 0; viewIdx < viewCount; ++viewIdx) {
			for (; ; ++next_vp){
				auto tins = capgDs<PWP3DInput>(dsname, next_vp);
				if (boost::filesystem::exists(tins.sSrcImage)){
					break;
				}
			}
			auto ins = capgDs<PWP3DInput>(dsname, next_vp);
			std::cout << "viewpoint: " << next_vp;
			++next_vp;

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

			//?? how to update??
			HistogramEngine::Instance()->UpdateVarBinHistogram(
				objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
				objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);

			// ---------------------------------------------------------------------------
			//iteration configuration for one object
			iterConfig->width = width; iterConfig->height = height;
			iterConfig->iterViewIds[viewIdx] = viewIdx; 
			iterConfig->iterObjectCount[viewIdx] = 1; //each view contains 1 object
			iterConfig->levelSetBandSize = 8;
			iterConfig->iterObjectIds[viewIdx][objectIdx] = 0;
			iterConfig->iterViewCount = viewCount; //to change
			iterConfig->iterCount = 10;

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
			//cv::waitKey(-1);

		}
		cout << "[App] Finish Rendered object initial pose." << endl;

		for (i = 3; i < 4; i++)
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
			break;
		}

		int viewIdx = 0;
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
			cv::imwrite(identifier_str + ".jpg", ResultMat);

			char large_buff[1000];
			sprintf(large_buff, "%f %f %f\n %f %f %f %f\n",
				objects[objectIdx]->pose[viewIdx]->translation->x,
				objects[objectIdx]->pose[viewIdx]->translation->y,
				objects[objectIdx]->pose[viewIdx]->translation->z,
				objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
				objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
				objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
				objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);

			std::ofstream out(identifier_str + ".txt");
			out << large_buff;

		//posteriors plot
		sprintf(str_result, (identifier_str + "_post.png").c_str());

		VisualisationEngine::Instance()->GetImage(
			ResultImage, GETIMAGE_POSTERIORS,
			objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

		ImageUtils::Instance()->SaveImageToFile(ResultImage, str_result);

		//primary engine destructor
		OptimisationEngine::Instance()->Shutdown();

		for (i = 0; i < objectCount; i++) delete objects[i];
		delete []objects;

		for (i = 0; i < viewCount; i++) delete views[i];
		delete views;

		delete ResultImage;

		cout << "Exit pwp3D app successfully." << endl;
	}
	return 0;
}

