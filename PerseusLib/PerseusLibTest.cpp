#include "../PerseusLib/PerseusLib.h"

#include "Utils/Timer.h"
#include <opencv2/opencv.hpp>
#include <armadillo>

using namespace Perseus::Utils;

std::string working_root = "C:/Users/Justin/workspace";
std::string project_root = working_root + "/PWP3D";

std::string sModelPath = project_root + "/Files/Models/Renderer/antenna.obj";

std::string sSrcImage = working_root +  "/remincline/data/fan/HuaweiData/data/mask/SJ21/pic_0.jpg";
//std::string sSrcImage = project_root + "/Files/Images/Red.png"; //

std::string sCameraMatrix = project_root +  "/Files/fan/cam/SJ21.cal"; //v
//std::string sCameraMatrix = project_root +  "/Files/CameraCalibration/900nc.cal"; //v

std::string sTargetMask = working_root + "/remincline/data/fan/HuaweiData/data/mask/SJ21/pic_0.bmp";
//std::string sTargetMask = project_root + "/Files/Masks/480p_All_VideoMask.png";

std::string sHistSrc = working_root + "/remincline/data/fan/HuaweiData/data/mask/SJ21/pic_0.bmp";
//std::string sHistSrc = project_root + "/Files/Masks/Red_Source.png";

//std::string sHistMask = project_root + "/Files/Masks/Red_Mask.png";
std::string sHistMask = working_root + "/remincline/data/fan/HuaweiData/data/mask/SJ21/pic_0.bmp";


std::string camrtpath = project_root +  "/Files/fan/cam/SJ21.rt.txt"; //v
std::string modelrtpath = project_root +  "/Files/fan/rt/SJ21.txt"; //v


template <typename MatType>
void describe(const MatType& T){
	std::cout << T.n_rows << "  " << T.n_cols << std::endl;
}





void init() {


	std::vector<arma::mat> mv_mats;
	{//load model-view r/t
		arma::mat camera_rts;
		arma::mat model_rt;
		camera_rts.load(camrtpath);
		model_rt.load(modelrtpath);
		std::cout << camera_rts.size() << std::endl <<camera_rts << std::endl;
		describe(camera_rts);

		std::cout << model_rt.size() << std::endl <<model_rt << std::endl;
		describe(model_rt);

		for (int iv = 0; iv < camera_rts.n_rows; iv += 4){
			arma::mat mv = camera_rts.rows(iv, iv + 3) * model_rt;
			mv_mats.push_back(mv);
		}
	}

	for (auto m : mv_mats){
		std::cout << m << std::endl;
	}

}


int main(void)
{
	init();

	char str[100];
	int i;

	// this is from image size
	int width = 1920, height = 1080;

	// fix view count
	int viewCount = 1;
	int objectCount = 1;

	int objectId = 0;
	int viewIdx = 0;
	int objectIdx = 0;

	Timer t;

	//result visualisation
	ImageUChar4* ResultImage = new ImageUChar4(width, height);

	// ---------------------------------------------------------------------------
	//input image
	//camera = 24 bit colour rgb
	ImageUChar4* camera = new ImageUChar4(width, height);
	ImageUtils::Instance()->LoadImageFromFile(camera, (char*)sSrcImage.c_str());

	//objects allocation + initialisation: 3d model in obj required
	Object3D **objects = new Object3D*[objectCount];

	std::cout << "\n==[APP] Init Model ==" << std::endl;
	objects[objectIdx] = new Object3D(objectId, viewCount, (char*)sModelPath.c_str(), width, height);

	// ---------------------------------------------------------------------------
	//views allocation + initialisation: camera calibration (artoolkit format) required
	std::cout << "\n==[APP] Init CameraMatrix ==" << std::endl;
	View3D **views = new View3D*[viewCount];
	views[viewIdx] = new View3D(0, (char*)sCameraMatrix.c_str(), width, height);


	// ---------------------------------------------------------------------------
	//histogram initialisation
	//source = 24 bit colour rgb
	//mask = 24 bit black/white png - white represents object
	//videoMask = 24 bit black/white png - white represents parts of the image that are usable
	std::cout << "\n==[APP] Init Target ROI ==" << std::endl;
	ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
		(char*)sTargetMask.c_str());

	ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
		(char*)sHistSrc.c_str());

	ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
		(char*)sHistMask.c_str(), objectIdx + 1);

	//?? how to update??
	HistogramEngine::Instance()->UpdateVarBinHistogram(
		objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
		objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


	// ---------------------------------------------------------------------------
	//iteration configuration for one object
	IterationConfiguration *iterConfig = new IterationConfiguration();
	iterConfig->width = width; iterConfig->height = height;
	iterConfig->iterViewIds[viewIdx] = 0; //?? how to set this??
	iterConfig->iterObjectCount[viewIdx] = 1;
	iterConfig->levelSetBandSize = 8;
	iterConfig->iterObjectIds[viewIdx][objectIdx] = 0; //???
	iterConfig->iterViewCount = 1; //to change
	iterConfig->iterCount = 1;

	//step size per object and view
	objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 0.5f, 0.5f, 10.0f);

	//initial pose per object and view
	// Notice the input pose here is angle, not radians for the rotation part
	//  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
	//        -1.98f, -2.90f, 37.47f, -40.90f, -207.77f, 27.48f);

	// for blue car demo
	//  objects[objectIdx]->initialPose[viewIdx]->SetFrom( -3.0f,-4.5f,28.f, -220.90f, -207.77f, 87.48f);

	// for red can demo
	// todo set fromMatrix

	objects[objectIdx]->initialPose[viewIdx]->SetFrom(
		-564.55370544,   423.54948248,  3065.63973947,//translation
		0.75586788,  0.00165458,  0.64490845, 0.11293404//rotation quaternion
		);

	//primary initilisation
	OptimisationEngine::Instance()->Initialise(width, height);

	//register camera image with main engine
	// todo ??? how many ??
	OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

	// ---------------------------------------------------------------------------
	std::cout << "\n==[APP] Rendering object initial pose.. ==" << std::endl;
	VisualisationEngine::Instance()->GetImage(
		ResultImage, GETIMAGE_PROXIMITY,
		objects[objectIdx], views[viewIdx],
		objects[objectIdx]->initialPose[viewIdx]);

	cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);
	cv::imshow("initial pose", ResultMat);
	cv::imwrite("test.jpg", ResultMat);

	std::cout << "[cam info]: ";
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			std::cout << views[viewIdx]->renderView->camera3D->K[i][j] << ' ';
		std::cout << std::endl;


	cv::waitKey(-1);

	std::cout << "[App] Finish Rendered object initial pose." << std::endl;

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

		printf("======= mode: useCUDAAEF: %d, use CUDARender %d ========;\n",
			iterConfig->useCUDAEF, iterConfig->useCUDARender);

		sprintf(str, "C:/Users/Justin/workspace/PWP3D/Files/Results/result%04d.png", i);

		//main processing
		t.restart();
		OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
		t.check("Iteration");

		//result plot
		VisualisationEngine::Instance()->GetImage(
			ResultImage, GETIMAGE_PROXIMITY,
			objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

		//result save to file
		//    ImageUtils::Instance()->SaveImageToFile(result, str);
		cv::Mat ResultMat(height, width, CV_8UC4, ResultImage->pixels);
		cv::imshow("result", ResultMat);
		cv::waitKey(2000);

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
	sprintf(str, "C:/Users/Justin/workspace/PWP3D/Files/Results/posteriors.png");
	VisualisationEngine::Instance()->GetImage(
		ResultImage, GETIMAGE_POSTERIORS,
		objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

	ImageUtils::Instance()->SaveImageToFile(ResultImage, str);

	//primary engine destructor
	OptimisationEngine::Instance()->Shutdown();

	for (i = 0; i < objectCount; i++) delete objects[i];
	delete objects;

	for (i = 0; i < viewCount; i++) delete views[i];
	delete views;

	delete ResultImage;

	std::cout << "Exit pwp3D app successfully." << std::endl;

	return 0;
}
