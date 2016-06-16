#include "OptimisationEngine.h"
#include <chrono>
#include <boost/format.hpp>
#include <PerseusLib/CUDA/CUDAEngine.h>
#include "PerseusLib/Utils/VisualisationEngine.h"
#include <string>
#include <opencv2/opencv.hpp>


using namespace PerseusLib::Optimiser;

OptimisationEngine* OptimisationEngine::instance;

OptimisationEngine::OptimisationEngine(void) { }
OptimisationEngine::~OptimisationEngine(void) { }

void OptimisationEngine::Initialise(int width, int height)
{
  int i;

  objects = new Object3D**[PERSEUS_MAX_VIEW_COUNT];
  views = new View3D*[PERSEUS_MAX_VIEW_COUNT];
  for (i=0; i<PERSEUS_MAX_VIEW_COUNT; i++) objects[i] = new Object3D*[PERSEUS_MAX_OBJECT_COUNT];

  objectCount = new int[PERSEUS_MAX_VIEW_COUNT];

  stepSizes = new StepSize3D*[10];
  for (i=0; i<10; i++) stepSizes[i] = new StepSize3D();

  energyFunction_standard = new EFStandard();

  this->SetPresetStepSizes();

  MathUtils::Instance()->ReadAndAllocateHeaviside(8192, "C:/Users/Justin/workspace/PWP3D/Files/Others/heaviside.txt");

  initialiseCUDA(width, height, MathUtils::Instance()->heavisideFunction, MathUtils::Instance()->heavisideSize);

}

void OptimisationEngine::Shutdown()
{
  int i;

  shutdownCUDA();

  for (i=0; i<10; i++) delete stepSizes[i];

  delete objectCount;

  delete []stepSizes;
  delete energyFunction_standard;

  MathUtils::Instance()->DeallocateHeaviside();

  delete instance;
}

void OptimisationEngine::SetPresetStepSizes()
{
	//// from large to small, multi rouded.

  stepSizes[0]->tX = -0.005f; stepSizes[0]->tY = -0.005f; stepSizes[0]->tZ = -0.005f; stepSizes[0]->r = -0.0008f;

  stepSizes[1]->tX = -0.003f; stepSizes[1]->tY = -0.003f; stepSizes[1]->tZ = -0.003f; stepSizes[1]->r = -0.0003f;
  stepSizes[2]->tX = -0.003f; stepSizes[2]->tY = -0.003f; stepSizes[2]->tZ = -0.003f; stepSizes[2]->r = -0.0003f;

  stepSizes[3]->tX = -0.002f; stepSizes[3]->tY = -0.002f; stepSizes[3]->tZ = -0.003f; stepSizes[3]->r = -0.0003f;
  stepSizes[4]->tX = -0.002f; stepSizes[4]->tY = -0.002f; stepSizes[4]->tZ = -0.003f; stepSizes[4]->r = -0.0003f;
  stepSizes[5]->tX = -0.002f; stepSizes[5]->tY = -0.002f; stepSizes[5]->tZ = -0.003f; stepSizes[5]->r = -0.0003f;

  stepSizes[6]->tX = -0.001f; stepSizes[6]->tY = -0.001f; stepSizes[6]->tZ = -0.002f; stepSizes[6]->r = -0.0002f;
  stepSizes[7]->tX = -0.001f; stepSizes[7]->tY = -0.001f; stepSizes[7]->tZ = -0.002f; stepSizes[7]->r = -0.0002f;


  stepSizes[8]->tX = -0.0005f; stepSizes[8]->tY = -0.0005f; stepSizes[8]->tZ = -0.0005f; stepSizes[8]->r = -0.0001f;
  stepSizes[9]->tX = -0.0005f; stepSizes[9]->tY = -0.0005f; stepSizes[9]->tZ = -0.0005f; stepSizes[9]->r = -0.0001f;
}

void OptimisationEngine::RegisterViewImage(View3D *view, ImageUChar4* image)
{
	ImageUtils::Instance()->Copy(image, view->imageRegistered);
	view->imageRegistered->UpdateGPUFromCPU();
}

void OptimisationEngine::Minimise(Object3D **objects, View3D **views, IterationConfiguration *iterConfig)
{
  int objectIdx, viewIdx, iterIdx;

  ///////////// initialization
  this->iterConfig = iterConfig;

  viewCount = iterConfig->iterViewCount;
  for (viewIdx=0; viewIdx<viewCount; viewIdx++)
  {
    this->views[viewIdx] = views[iterConfig->iterViewIds[viewIdx]];
    this->objectCount[viewIdx] = iterConfig->iterObjectCount[viewIdx];
  }

  for (viewIdx=0; viewIdx<viewCount; viewIdx++) for (objectIdx=0; objectIdx<objectCount[viewIdx]; objectIdx++)
  {
    this->objects[viewIdx][objectIdx] = objects[iterConfig->iterObjectIds[viewIdx][objectIdx]];
    this->objects[viewIdx][objectIdx]->initialPose[viewIdx]->CopyInto(this->objects[viewIdx][objectIdx]->pose[viewIdx]);
    this->objects[viewIdx][objectIdx]->UpdateRendererFromPose(views[viewIdx]);
  }

  /////////// energey Function
  energyFunction = energyFunction_standard;

  ////////// rouned iteration
  for (iterIdx=0; iterIdx< iterConfig->iterCount; iterIdx++)
  {
	  std::cout << "[opt:] " << "round" << ' ' << iterIdx << " begin"<< std::endl;
	  this->RunOneMultiIteration(iterConfig);

	  {
		  auto tmili = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::system_clock::now().time_since_epoch()).count();
		  auto filename = boost::str(boost::format("%s_%d_iter_%d_%s") % iterConfig->idstr % iterIdx % 0 % std::to_string(tmili));

		  {
			  ImageUChar4* ResultImage = new ImageUChar4(1920, 1080);
			  VisualisationEngine::Instance()->GetImage(ResultImage, GETIMAGE_DT, objects[0], views[0], objects[0]->pose[0]);
			  cv::Mat ResultMat(1080, 1920, CV_8UC4, ResultImage->pixels);

			  cv::imwrite(filename + ".png", ResultMat);
			  delete ResultImage;
		  }

		  {
			  ImageUChar4* ResultImage = new ImageUChar4(1920, 1080);
			  VisualisationEngine::Instance()->GetImage(ResultImage, GETIMAGE_SIHLUETTE, objects[0], views[0], objects[0]->pose[0]);
			  cv::Mat ResultMat2(1080, 1920, CV_8UC4, ResultImage->pixels);

			  cv::imwrite(filename + "_sil.png", ResultMat2);
			  delete ResultImage;
		  }
	  }

	  std::cout << "[opt:] " << "round" << ' ' << iterIdx << " end" << std::endl;

  }
}


void OptimisationEngine::MinimiseSingle(Object3D **objects, View3D **views, IterationConfiguration *iterConfig, int nIterNum)
{
	int objectIdx, viewIdx, iterIdx;

	this->iterConfig = iterConfig;

	viewCount = iterConfig->iterViewCount;
	for (viewIdx = 0; viewIdx < viewCount; viewIdx++)
	{
		this->views[viewIdx] = views[iterConfig->iterViewIds[viewIdx]];
		this->objectCount[viewIdx] = iterConfig->iterObjectCount[viewIdx];
	}

	for (viewIdx = 0; viewIdx < viewCount; viewIdx++) for (objectIdx = 0; objectIdx < objectCount[viewIdx]; objectIdx++)
	{
		this->objects[viewIdx][objectIdx] = objects[iterConfig->iterObjectIds[viewIdx][objectIdx]];
		this->objects[viewIdx][objectIdx]->initialPose[viewIdx]->CopyInto(this->objects[viewIdx][objectIdx]->pose[viewIdx]);
		this->objects[viewIdx][objectIdx]->UpdateRendererFromPose(views[viewIdx]);
	}

	energyFunction = energyFunction_standard;

	for (iterIdx = 0; iterIdx < iterConfig->iterCount; iterIdx++)
	{
		if (nIterNum > 7 || nIterNum < 0){
			std::cout << "fatal error! nIterNum must be from 0 to 7" << std::endl;
			exit(-1);
		}

		this->RunOneSingleIteration(stepSizes[nIterNum], iterConfig); if (this->HasConverged()) return;
	}
}

void OptimisationEngine::RunOneMultiIteration(IterationConfiguration* iterConfig)
{
	this->RunOneSingleIteration(stepSizes[0], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[1], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[2], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[3], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[4], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[5], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[6], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[7], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[8], iterConfig); if (this->HasConverged()) return;
	this->RunOneSingleIteration(stepSizes[9], iterConfig); if (this->HasConverged()) return;

	this->NormaliseRotation();
}

void OptimisationEngine::RunOneSingleIteration(StepSize3D* presetStepSize, IterationConfiguration* iterConfig)
{
	energyFunction->PrepareIteration(objects, objectCount, views, viewCount, iterConfig);

	// update the pose of the object
	energyFunction->GetFirstDerivativeValues(objects, objectCount, views, viewCount, iterConfig);

	this->DescendWithGradient(presetStepSize, iterConfig);

	if(false){
		auto tmili = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::system_clock::now().time_since_epoch()).count();
		auto filename = boost::str(boost::format("iter_%s") % std::to_string(tmili));

		ImageUChar4* ResultImage = new ImageUChar4(1920, 1080);

		VisualisationEngine::Instance()->GetImage(ResultImage, GETIMAGE_DT, objects[0][0], views[0], objects[0][0]->pose[0]);
		  cv::Mat ResultMat(1080, 1920, CV_8UC4, ResultImage->pixels);

		  cv::imwrite(filename + ".png", ResultMat);
		  delete ResultImage;

		  ResultImage = new ImageUChar4(1920, 1080);
		  VisualisationEngine::Instance()->GetImage(ResultImage, GETIMAGE_SIHLUETTE, objects[0][0], views[0], objects[0][0]->pose[0]);
		  cv::Mat ResultMat2(1080, 1920, CV_8UC4, ResultImage->pixels);

		  cv::imwrite(filename + "_sil.png", ResultMat2);
		  delete ResultImage;
	  }
}

void OptimisationEngine::DescendWithGradient(StepSize3D *presetStepSize, IterationConfiguration *iterConfig)
{
  int objectIdx, viewIdx;

  StepSize3D actualStepSize;

  for (viewIdx = 0; viewIdx < viewCount; viewIdx++) for (objectIdx = 0; objectIdx < objectCount[viewIdx]; objectIdx++)
  {
    actualStepSize.r = presetStepSize->r * objects[viewIdx][objectIdx]->stepSize[viewIdx]->r;
    actualStepSize.tX = presetStepSize->tX * objects[viewIdx][objectIdx]->stepSize[viewIdx]->tX;
    actualStepSize.tY = presetStepSize->tY * objects[viewIdx][objectIdx]->stepSize[viewIdx]->tY;
    actualStepSize.tZ = presetStepSize->tZ * objects[viewIdx][objectIdx]->stepSize[viewIdx]->tZ;

	if (isfinite(objects[viewIdx][objectIdx]->dpose[views[viewIdx]->viewId]->translation->z)==false){
		bool flag = false;




	}



	if (isfinite(objects[viewIdx][objectIdx]->dpose[views[viewIdx]->viewId]->rotation->vector4d.z)==false){
		bool flag = false;
	}


    switch (iterConfig->iterTarget[0])
    {
    case ITERATIONTARGET_BOTH:
      AdvanceTranslation(objects[viewIdx][objectIdx], views[viewIdx], &actualStepSize);

      AdvanceRotation(objects[viewIdx][objectIdx], views[viewIdx], &actualStepSize);

      break;
    case ITERATIONTARGET_TRANSLATION:
      AdvanceTranslation(objects[viewIdx][objectIdx], views[viewIdx], &actualStepSize);
      break;
    case ITERATIONTARGET_ROTATION:
      AdvanceRotation(objects[viewIdx][objectIdx], views[viewIdx], &actualStepSize);
      break;
    }

    objects[viewIdx][objectIdx]->UpdateRendererFromPose(views[viewIdx]);
  }
}
void OptimisationEngine::AdvanceTranslation(Object3D* object, View3D* view, StepSize3D* stepSize)
{
  object->pose[view->viewId]->translation->x -= stepSize->tX * object->dpose[view->viewId]->translation->x;
  object->pose[view->viewId]->translation->y -= stepSize->tY * object->dpose[view->viewId]->translation->y;
  object->pose[view->viewId]->translation->z -= stepSize->tZ * object->dpose[view->viewId]->translation->z;
}
void OptimisationEngine::AdvanceRotation(Object3D* object, View3D* view, StepSize3D* stepSize)
{
  object->pose[view->viewId]->rotation->vector4d.x -= stepSize->r * object->dpose[view->viewId]->rotation->vector4d.x;
  object->pose[view->viewId]->rotation->vector4d.y -= stepSize->r * object->dpose[view->viewId]->rotation->vector4d.y;
  object->pose[view->viewId]->rotation->vector4d.z -= stepSize->r * object->dpose[view->viewId]->rotation->vector4d.z;
  object->pose[view->viewId]->rotation->vector4d.w -= stepSize->r * object->dpose[view->viewId]->rotation->vector4d.w;
}

void OptimisationEngine::NormaliseRotation()
{
  int objectIdx, viewIdx;
  for (viewIdx = 0; viewIdx < viewCount; viewIdx++) for (objectIdx = 0; objectIdx < objectCount[viewIdx]; objectIdx++)
  {
	auto pose = objects[viewIdx][objectIdx]->pose[viewIdx];
	auto R = pose->rotation;
	auto T = pose->translation;
    objects[viewIdx][objectIdx]->pose[viewIdx]->rotation->Normalize();
	pose = objects[viewIdx][objectIdx]->pose[viewIdx];
    objects[viewIdx][objectIdx]->UpdateRendererFromPose(views[viewIdx]);
  }
}

bool OptimisationEngine::HasConverged()
{
  return false;
}
