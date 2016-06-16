#include "EFStandard.h"
#include <boost/format.hpp>
#include <chrono>
#include <string>
#include "PerseusLib/Utils/VisualisationEngine.h"
#include <opencv2/opencv.hpp>

using namespace PerseusLib::Optimiser;

#include <PerseusLib/CUDA/CUDAEngine.h>

#include <PerseusLib/Utils/ImageUtils.h>
using namespace PerseusLib::Utils;


EFStandard::EFStandard(void)
{
}

EFStandard::~EFStandard(void)
{
}

void EFStandard::PrepareIteration(Object3D ***objects, int *objectCount, View3D** views, int viewCount, IterationConfiguration* iterConfig)
{
	Object3D* object; View3D* view; int objectIdx, viewIdx;

	for (viewIdx = 0; viewIdx < viewCount; viewIdx++)
	{
		view = views[viewIdx];

		// draw all in the view
		if (objectCount[viewIdx] > 1) DrawingEngine::Instance()->DrawAllInView(objects[viewIdx], objectCount[viewIdx], view, iterConfig->useCUDARender, true);

		for (objectIdx = 0; objectIdx < objectCount[viewIdx]; objectIdx++)
		{
			object = objects[viewIdx][objectIdx];

			// render object
			DrawingEngine::Instance()->Draw(object, view, iterConfig->useCUDARender, !iterConfig->useCUDAEF);
			DrawingEngine::Instance()->ChangeROIWithBand(object, view, iterConfig->levelSetBandSize, iterConfig->width, iterConfig->height);

			registerObjectImage(object, view, iterConfig->useCUDARender, (objectCount[viewIdx] > 1));

			processDTSihluetteLSDXDY(object, view, iterConfig->levelSetBandSize);

			auto tmili = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::system_clock::now().time_since_epoch()).count();
			auto filename = boost::str(boost::format("%d_%s.png") % viewIdx% std::to_string(tmili));
			/*
			  ImageUChar4* ResultImage = new ImageUChar4(1920, 1080);
			  VisualisationEngine::Instance()->GetImage(ResultImage, GETIMAGE_DT, object, views[0], object->pose[0]);

			  cv::Mat ResultMat(1080, 1920, CV_8UC4, ResultImage->pixels);

			  cv::imwrite(filename, ResultMat);
			  delete ResultImage;
			  */
		}
	}
}

void EFStandard::GetFirstDerivativeValues(Object3D ***objects, int *objectCount, View3D** views, int viewCount, IterationConfiguration* iterConfig)
{
	int objectIdx, viewIdx;
	Object3D* object; View3D* view;

	if (iterConfig->useCUDAEF)
	{
		for (viewIdx = 0; viewIdx < viewCount; viewIdx++) for (objectIdx = 0; objectIdx < objectCount[viewIdx]; objectIdx++)
		{
			object = objects[viewIdx][objectIdx]; view = views[viewIdx];

			registerObjectAndViewGeometricData(object, view);

			processAndGetEFFirstDerivatives(object, view, (objectCount[viewIdx] > 1));
		}
		return;
	}

	this->GetFirstDerivativeValues_CPU_6DoF(objects, objectCount, views, viewCount, iterConfig);
}

void EFStandard::GetFirstDerivativeValues_CPU_6DoF(Object3D ***objects, int *objectCount, View3D** views, int viewCount, IterationConfiguration* iterConfig)
{
	int objectIdx, viewIdx, objectId, viewId;
	Object3D* object; View3D* view;

	int width = iterConfig->width, height = iterConfig->height;

	int i, j, k, idx, icX, icY, icZ, nhidx;
	float pYB, pYF, dtIdx, dfPPGeneric, dirac, heaviside;
	unsigned char r, b, g;
	int *dtPosX, *dtPosY;
	float *dt, *dtDX, *dtDY;
	float xProjected[4], xUnprojected[4], xUnrotated[4], dfPP[7], dpose[7], otherInfo[2];

	for (viewIdx = 0; viewIdx < viewCount; viewIdx++) for (objectIdx = 0; objectIdx < objectCount[viewIdx]; objectIdx++)
	{
		view = views[viewIdx]; object = objects[viewIdx][objectIdx];

		viewId = view->viewId; objectId = object->objectId;

		dt = object->dt[viewId]->pixels;
		dtPosX = object->dtPosX[viewId]->pixels; dtPosY = object->dtPosY[viewId]->pixels;
		dtDX = object->dtDX[viewId]->pixels; dtDY = object->dtDY[viewId]->pixels;

		getProcessedDataDTSihluetteLSDXDY(object, view);

		// init dpose to 0
		for (i = 0; i < 7; i++) dpose[i] = 0;

		// for each pixel in the image
		for (j = 0, idx = 0; j < height; j++) for (i = 0; i < width; idx++, i++)
		{
			if (dtPosY[idx] >= 0)// && view->videoMask->pixels[idx] > 128)
			{
				dtIdx = dt[idx];

				icX = i; icY = j;
				if (dtIdx < 0) { // background, then find the nearest
					icX = dtPosX[idx] + object->roiGenerated[viewId][0];
					icY = dtPosY[idx] + object->roiGenerated[viewId][1];
				}
				icZ = icX + icY * width;

				if (objectCount[viewIdx] > 1)
					if (((view->imageRenderAll->imageObjects->pixels[icZ] - 1) != objectId) ||
						((view->imageRenderAll->imageObjects->pixels[i + j * width] - 1) != objectId && (view->imageRenderAll->imageObjects->pixels[i + j * width] - 1) != -1))
						continue;

				nhidx = int(4096 + 512 * dtIdx);
				// for a vaild pixel
				if (nhidx >= 0 && nhidx < MathUtils::Instance()->heavisideSize)
				{
					heaviside = MathUtils::Instance()->heavisideFunction[nhidx];

					r = view->imageRegistered->pixels[idx].x; g = view->imageRegistered->pixels[idx].y; b = view->imageRegistered->pixels[idx].z;

					// object->histogramVarBin[viewId]->GetValue(&pYF, &pYB, r, g, b, i, j);

					pYF = 1.0f;
					pYB = 0.f;
					if (r + g + b < 15){ // back ground for sure.
						std::swap(pYF, pYB);
					}
					else if (r + g + b < 240 * 3){ // hard to say
						pYF = pYB = 0.5;
					}


					pYF += 0.0000001f; pYB += 0.0000001f;

					dirac = (1.0f / float(PI)) * (1.0f / (dtIdx * dtIdx + 1.0f) + float(1e-3));

					// Get the derivate of PWP3D. this is for update
					dfPPGeneric = dirac * (pYF - pYB) / (heaviside * (pYF - pYB) + pYB);

					/// ------- run 1
					xProjected[0] = (float)2 * (icX - view->renderView->view[0]) / view->renderView->view[2] - 1.0f;
					xProjected[1] = (float)2 * (icY - view->renderView->view[1]) / view->renderView->view[3] - 1.0f;
					xProjected[2] = (float)2 * ((float)object->imageRender[viewId]->imageZBuffer->pixels[icZ] / (float)MAX_INT) - 1.0f;
					xProjected[3] = 1.0f;


					if (fabs(xProjected[2] - 1) < 1e-16){

						auto tmili = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::system_clock::now().time_since_epoch()).count();
						auto filename = boost::str(boost::format("%d_%s") % viewIdx% std::to_string(tmili));

						ImageUChar4* ResultImage = new ImageUChar4(1920, 1080);
						VisualisationEngine::Instance()->GetImage(ResultImage, GETIMAGE_DT, object, view, object->pose[0]);

						cv::Mat ResultMat(1080, 1920, CV_8UC4, ResultImage->pixels);

						cv::imwrite(filename + ".png", ResultMat);
						delete ResultImage;

						ResultImage = new ImageUChar4(1920, 1080);
						VisualisationEngine::Instance()->GetImage(ResultImage, GETIMAGE_SIHLUETTE, object, view, object->pose[0]);
						cv::Mat ResultMat2(1080, 1920, CV_8UC4, ResultImage->pixels);

						cv::imwrite(filename + "_sil.png", ResultMat2);
						delete ResultImage;


						bool root = true;
						cv::Mat depth(height, width, CV_32FC1);
						for (auto ii = 0; ii < height; ++ii){
							for (auto jj = 0; jj < width; ++jj){
								depth.at<float>(ii, jj) = object->imageRender[viewId]->imageZBuffer->pixels[ii*width + jj];
							}
						}

						cv::Mat dst;
						double minv, maxv;
						cv::minMaxIdx(depth, &minv, &maxv);
						depth -= minv;
						depth.convertTo(dst, CV_8U, 255.0 / (maxv - minv));

						cv::imwrite(filename + "_depth.png", dst);

						continue;
					}

					MathUtils::Instance()->MatrixVectorProduct4(view->renderView->invP, xProjected, xUnprojected);
					MathUtils::Instance()->MatrixVectorProduct4(object->invPMMatrix[viewId], xProjected, xUnrotated);

					otherInfo[0] = view->renderView->projectionParams.A * dtDX[idx];
					otherInfo[1] = view->renderView->projectionParams.B * dtDY[idx];

					dfPP[0] = -otherInfo[0] / xUnprojected[2];
					dfPP[1] = -otherInfo[1] / xUnprojected[2];
					dfPP[2] = (otherInfo[0] * xUnprojected[0] + otherInfo[1] * xUnprojected[1]) / (xUnprojected[2] * xUnprojected[2]);

					object->renderObject->objectCoordinateTransform[viewId]->rotation->GetDerivatives(dfPP + 3, xUnprojected, xUnrotated,
						view->renderView->projectionParams.all, otherInfo);

					for (k = 0; k < 7; k++) {
						dfPP[k] *= dfPPGeneric;
						dpose[k] += dfPP[k];
					}

					/// -------- run 2
					xProjected[0] = (float)2 * (icX - view->renderView->view[0]) / view->renderView->view[2] - 1.0f;
					xProjected[1] = (float)2 * (icY - view->renderView->view[1]) / view->renderView->view[3] - 1.0f;
					xProjected[2] = (float)2 * ((float)object->imageRender[viewId]->imageZBufferInverse->pixels[icZ] / (float)MAX_INT) - 1.0f;
					xProjected[3] = 1.0f;

					MathUtils::Instance()->MatrixVectorProduct4(view->renderView->invP, xProjected, xUnprojected);
					MathUtils::Instance()->MatrixVectorProduct4(object->invPMMatrix[viewId], xProjected, xUnrotated);

					otherInfo[0] = view->renderView->projectionParams.A * dtDX[idx];
					otherInfo[1] = view->renderView->projectionParams.B * dtDY[idx];

					dfPP[0] = -otherInfo[0] / xUnprojected[2];
					dfPP[1] = -otherInfo[1] / xUnprojected[2];
					dfPP[2] = (otherInfo[0] * xUnprojected[0] + otherInfo[1] * xUnprojected[1]) / (xUnprojected[2] * xUnprojected[2]);

					object->renderObject->objectCoordinateTransform[viewId]->rotation->GetDerivatives(dfPP + 3, xUnprojected, xUnrotated,
						view->renderView->projectionParams.all, otherInfo);

					for (k = 0; k < 7; k++) {
						dfPP[k] *= dfPPGeneric;
						dpose[k] += dfPP[k];
					}

				}
			}
		}

		object->dpose[viewId]->SetFrom(dpose, 7);

		char rez[200];
		sprintf(rez, "%4.5f %4.5f %4.5f %4.5f %4.5f %4.5f %4.5f", object->dpose[viewId]->translation->x, object->dpose[viewId]->translation->y,
			object->dpose[viewId]->translation->z, object->dpose[viewId]->rotation->vector4d.x, object->dpose[viewId]->rotation->vector4d.y,
			object->dpose[viewId]->rotation->vector4d.z, object->dpose[viewId]->rotation->vector4d.w);

		//DEBUGBREAK;
	}
}
