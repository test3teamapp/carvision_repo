/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "detectNet.h"
#include "tensorConvert.h"

#include "cudaMappedMemory.h"
#include "cudaFont.h"

//cang
#include "cudaDraw.h"

#include "commandLine.h"
#include "filesystem.h"
#include "logging.h"

#define OUTPUT_CVG 0  // Caffe has output coverage (confidence) heat map
#define OUTPUT_BBOX 1 // Caffe has separate output layer for bounding box

#define OUTPUT_UFF 0 // UFF has primary output containing detection results
#define OUTPUT_NUM 1 // UFF has secondary output containing one detection count

#define OUTPUT_CONF 0 // ONNX SSD-Mobilenet has confidence as first, bbox second

#define CHECK_NULL_STR(x) (x != NULL) ? x : "NULL"
//#define DEBUG_CLUSTERING

// constructor
detectNet::detectNet(float meanPixel) : tensorNet()
{
	mCoverageThreshold = DETECTNET_DEFAULT_THRESHOLD;
	mMeanPixel = meanPixel;
	mNumClasses = 0;

	mClassColors[0] = NULL; // cpu ptr
	mClassColors[1] = NULL; // gpu ptr

	mDetectionSets[0] = NULL; // cpu ptr
	mDetectionSets[1] = NULL; // gpu ptr
	mDetectionSet = 0;
	mMaxDetections = 0;
}

// destructor
detectNet::~detectNet()
{
	if (mDetectionSets != NULL)
	{
		CUDA(cudaFreeHost(mDetectionSets[0]));

		mDetectionSets[0] = NULL;
		mDetectionSets[1] = NULL;
	}

	if (mClassColors != NULL)
	{
		CUDA(cudaFreeHost(mClassColors[0]));

		mClassColors[0] = NULL;
		mClassColors[1] = NULL;
	}
}

// init
bool detectNet::init(const char *prototxt, const char *model, const char *mean_binary, const char *class_labels,
					 float threshold, const char *input_blob, const char *coverage_blob, const char *bbox_blob,
					 uint32_t maxBatchSize, precisionType precision, deviceType device, bool allowGPUFallback)
{
	LogInfo("\n");
	LogInfo("detectNet -- loading detection network model from:\n");
	LogInfo("          -- prototxt     %s\n", CHECK_NULL_STR(prototxt));
	LogInfo("          -- model        %s\n", CHECK_NULL_STR(model));
	LogInfo("          -- input_blob   '%s'\n", CHECK_NULL_STR(input_blob));
	LogInfo("          -- output_cvg   '%s'\n", CHECK_NULL_STR(coverage_blob));
	LogInfo("          -- output_bbox  '%s'\n", CHECK_NULL_STR(bbox_blob));
	LogInfo("          -- mean_pixel   %f\n", mMeanPixel);
	LogInfo("          -- mean_binary  %s\n", CHECK_NULL_STR(mean_binary));
	LogInfo("          -- class_labels %s\n", CHECK_NULL_STR(class_labels));
	LogInfo("          -- threshold    %f\n", threshold);
	LogInfo("          -- batch_size   %u\n\n", maxBatchSize);

	// create list of output names
	std::vector<std::string> output_blobs;

	if (coverage_blob != NULL)
		output_blobs.push_back(coverage_blob);

	if (bbox_blob != NULL)
		output_blobs.push_back(bbox_blob);

	// ONNX SSD models require larger workspace size
	if (modelTypeFromPath(model) == MODEL_ONNX)
	{
		size_t gpuMemFree = 0;
		size_t gpuMemTotal = 0;

		CUDA(cudaMemGetInfo(&gpuMemFree, &gpuMemTotal));

		if (gpuMemTotal <= (2048 << 20))
			mWorkspaceSize = 512 << 20;
		else
			mWorkspaceSize = 2048 << 20;
	}

	// load the model
	if (!LoadNetwork(prototxt, model, mean_binary, input_blob, output_blobs,
					 maxBatchSize, precision, device, allowGPUFallback))
	{
		LogError(LOG_TRT "detectNet -- failed to initialize.\n");
		return false;
	}

	// allocate detection sets
	if (!allocDetections())
		return false;

	// load class descriptions
	loadClassInfo(class_labels);

	// set default class colors
	if (!defaultColors())
		return false;

	// set the specified threshold
	SetThreshold(threshold);

	return true;
}

// Create
detectNet *detectNet::Create(const char *prototxt, const char *model, float mean_pixel, const char *class_labels,
							 float threshold, const char *input_blob, const char *coverage_blob, const char *bbox_blob,
							 uint32_t maxBatchSize, precisionType precision, deviceType device, bool allowGPUFallback)
{
	detectNet *net = new detectNet(mean_pixel);

	if (!net)
		return NULL;

	if (!net->init(prototxt, model, NULL, class_labels, threshold, input_blob, coverage_blob, bbox_blob,
				   maxBatchSize, precision, device, allowGPUFallback))
		return NULL;

	return net;
}

// Create
detectNet *detectNet::Create(const char *prototxt, const char *model, const char *mean_binary, const char *class_labels,
							 float threshold, const char *input_blob, const char *coverage_blob, const char *bbox_blob,
							 uint32_t maxBatchSize, precisionType precision, deviceType device, bool allowGPUFallback)
{
	detectNet *net = new detectNet();

	if (!net)
		return NULL;

	if (!net->init(prototxt, model, mean_binary, class_labels, threshold, input_blob, coverage_blob, bbox_blob,
				   maxBatchSize, precision, device, allowGPUFallback))
		return NULL;

	return net;
}

// Create (UFF)
detectNet *detectNet::Create(const char *model, const char *class_labels, float threshold,
							 const char *input, const Dims3 &inputDims,
							 const char *output, const char *numDetections,
							 uint32_t maxBatchSize, precisionType precision,
							 deviceType device, bool allowGPUFallback)
{
	detectNet *net = new detectNet();

	if (!net)
		return NULL;

	LogInfo("\n");
	LogInfo("detectNet -- loading detection network model from:\n");
	LogInfo("          -- model        %s\n", CHECK_NULL_STR(model));
	LogInfo("          -- input_blob   '%s'\n", CHECK_NULL_STR(input));
	LogInfo("          -- output_blob  '%s'\n", CHECK_NULL_STR(output));
	LogInfo("          -- output_count '%s'\n", CHECK_NULL_STR(numDetections));
	LogInfo("          -- class_labels %s\n", CHECK_NULL_STR(class_labels));
	LogInfo("          -- threshold    %f\n", threshold);
	LogInfo("          -- batch_size   %u\n\n", maxBatchSize);

	// create list of output names
	std::vector<std::string> output_blobs;

	if (output != NULL)
		output_blobs.push_back(output);

	if (numDetections != NULL)
		output_blobs.push_back(numDetections);

	// load the model
	if (!net->LoadNetwork(NULL, model, NULL, input, inputDims, output_blobs,
						  maxBatchSize, precision, device, allowGPUFallback))
	{
		LogError(LOG_TRT "detectNet -- failed to initialize.\n");
		return NULL;
	}

	// allocate detection sets
	if (!net->allocDetections())
		return NULL;

	// load class descriptions
	net->loadClassInfo(class_labels);

	// set default class colors
	if (!net->defaultColors())
		return NULL;

	// set the specified threshold
	net->SetThreshold(threshold);

	return net;
}

// Create
detectNet *detectNet::Create(NetworkType networkType, float threshold, uint32_t maxBatchSize,
							 precisionType precision, deviceType device, bool allowGPUFallback)
{
#if 1
	if (networkType == PEDNET_MULTI)
		return Create("networks/multiped-500/deploy.prototxt", "networks/multiped-500/snapshot_iter_178000.caffemodel", 117.0f, "networks/multiped-500/class_labels.txt", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == FACENET)
		return Create("networks/facenet-120/deploy.prototxt", "networks/facenet-120/snapshot_iter_24000.caffemodel", 0.0f, "networks/facenet-120/class_labels.txt", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == PEDNET)
		return Create("networks/ped-100/deploy.prototxt", "networks/ped-100/snapshot_iter_70800.caffemodel", 0.0f, "networks/ped-100/class_labels.txt", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_AIRPLANE)
		return Create("networks/DetectNet-COCO-Airplane/deploy.prototxt", "networks/DetectNet-COCO-Airplane/snapshot_iter_22500.caffemodel", 0.0f, "networks/DetectNet-COCO-Airplane/class_labels.txt", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_BOTTLE)
		return Create("networks/DetectNet-COCO-Bottle/deploy.prototxt", "networks/DetectNet-COCO-Bottle/snapshot_iter_59700.caffemodel", 0.0f, "networks/DetectNet-COCO-Bottle/class_labels.txt", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_CHAIR)
		return Create("networks/DetectNet-COCO-Chair/deploy.prototxt", "networks/DetectNet-COCO-Chair/snapshot_iter_89500.caffemodel", 0.0f, "networks/DetectNet-COCO-Chair/class_labels.txt", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_DOG)
		return Create("networks/DetectNet-COCO-Dog/deploy.prototxt", "networks/DetectNet-COCO-Dog/snapshot_iter_38600.caffemodel", 0.0f, "networks/DetectNet-COCO-Dog/class_labels.txt", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
#if NV_TENSORRT_MAJOR > 4
	else if (networkType == SSD_INCEPTION_V2)
		return Create("networks/SSD-Inception-v2/ssd_inception_v2_coco.uff", "networks/SSD-Inception-v2/ssd_coco_labels.txt", threshold, "Input", Dims3(3, 300, 300), "NMS", "NMS_1", maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == SSD_MOBILENET_V1)
		return Create("networks/SSD-Mobilenet-v1/ssd_mobilenet_v1_coco.uff", "networks/SSD-Mobilenet-v1/ssd_coco_labels.txt", threshold, "Input", Dims3(3, 300, 300), "Postprocessor", "Postprocessor_1", maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == SSD_MOBILENET_V2)
		return Create("networks/SSD-Mobilenet-v2/ssd_mobilenet_v2_coco.uff", "networks/SSD-Mobilenet-v2/ssd_coco_labels.txt", threshold, "Input", Dims3(3, 300, 300), "NMS", "NMS_1", maxBatchSize, precision, device, allowGPUFallback);
#endif
	else
		return NULL;
#else
	if (networkType == PEDNET_MULTI)
		return Create("networks/multiped-500/deploy.prototxt", "networks/multiped-500/snapshot_iter_178000.caffemodel", "networks/multiped-500/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == FACENET)
		return Create("networks/facenet-120/deploy.prototxt", "networks/facenet-120/snapshot_iter_24000.caffemodel", NULL, threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == PEDNET)
		return Create("networks/ped-100/deploy.prototxt", "networks/ped-100/snapshot_iter_70800.caffemodel", "networks/ped-100/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_AIRPLANE)
		return Create("networks/DetectNet-COCO-Airplane/deploy.prototxt", "networks/DetectNet-COCO-Airplane/snapshot_iter_22500.caffemodel", "networks/DetectNet-COCO-Airplane/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_BOTTLE)
		return Create("networks/DetectNet-COCO-Bottle/deploy.prototxt", "networks/DetectNet-COCO-Bottle/snapshot_iter_59700.caffemodel", "networks/DetectNet-COCO-Bottle/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_CHAIR)
		return Create("networks/DetectNet-COCO-Chair/deploy.prototxt", "networks/DetectNet-COCO-Chair/snapshot_iter_89500.caffemodel", "networks/DetectNet-COCO-Chair/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else if (networkType == COCO_DOG)
		return Create("networks/DetectNet-COCO-Dog/deploy.prototxt", "networks/DetectNet-COCO-Dog/snapshot_iter_38600.caffemodel", "networks/DetectNet-COCO-Dog/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize, precision, device, allowGPUFallback);
	else
		return NULL;
#endif
}

// NetworkTypeFromStr
detectNet::NetworkType detectNet::NetworkTypeFromStr(const char *modelName)
{
	if (!modelName)
		return detectNet::CUSTOM;

	detectNet::NetworkType type = detectNet::PEDNET;

	if (strcasecmp(modelName, "multiped") == 0 || strcasecmp(modelName, "multiped-500") == 0)
		type = detectNet::PEDNET_MULTI;
	else if (strcasecmp(modelName, "pednet") == 0 || strcasecmp(modelName, "ped-100") == 0)
		type = detectNet::PEDNET;
	else if (strcasecmp(modelName, "facenet") == 0 || strcasecmp(modelName, "facenet-120") == 0 || strcasecmp(modelName, "face-120") == 0)
		type = detectNet::FACENET;
	else if (strcasecmp(modelName, "coco-airplane") == 0 || strcasecmp(modelName, "airplane") == 0)
		type = detectNet::COCO_AIRPLANE;
	else if (strcasecmp(modelName, "coco-bottle") == 0 || strcasecmp(modelName, "bottle") == 0)
		type = detectNet::COCO_BOTTLE;
	else if (strcasecmp(modelName, "coco-chair") == 0 || strcasecmp(modelName, "chair") == 0)
		type = detectNet::COCO_CHAIR;
	else if (strcasecmp(modelName, "coco-dog") == 0 || strcasecmp(modelName, "dog") == 0)
		type = detectNet::COCO_DOG;
#if NV_TENSORRT_MAJOR > 4
	else if (strcasecmp(modelName, "ssd-inception") == 0 || strcasecmp(modelName, "ssd-inception-v2") == 0 || strcasecmp(modelName, "coco-ssd-inception") == 0 || strcasecmp(modelName, "coco-ssd-inception-v2") == 0)
		type = detectNet::SSD_INCEPTION_V2;
	else if (strcasecmp(modelName, "ssd-mobilenet-v1") == 0 || strcasecmp(modelName, "coco-ssd-mobilenet-v1") == 0)
		type = detectNet::SSD_MOBILENET_V1;
	else if (strcasecmp(modelName, "ssd-mobilenet-v2") == 0 || strcasecmp(modelName, "coco-ssd-mobilenet-v2") == 0 || strcasecmp(modelName, "ssd-mobilenet") == 0)
		type = detectNet::SSD_MOBILENET_V2;
#endif
	else
		type = detectNet::CUSTOM;

	return type;
}

// Create
detectNet *detectNet::Create(int argc, char **argv)
{
	return Create(commandLine(argc, argv));
}

// Create
detectNet *detectNet::Create(const commandLine &cmdLine)
{
	detectNet *net = NULL;

	// parse command line parameters
	const char *modelName = cmdLine.GetString("network");

	if (!modelName)
		modelName = cmdLine.GetString("model", "ssd-mobilenet-v2");

	float threshold = cmdLine.GetFloat("threshold");

	if (threshold == 0.0f)
		threshold = DETECTNET_DEFAULT_THRESHOLD;

	int maxBatchSize = cmdLine.GetInt("batch_size");

	if (maxBatchSize < 1)
		maxBatchSize = DEFAULT_MAX_BATCH_SIZE;

	// parse the model type
	const detectNet::NetworkType type = NetworkTypeFromStr(modelName);

	if (type == detectNet::CUSTOM)
	{
		const char *prototxt = cmdLine.GetString("prototxt");
		const char *input = cmdLine.GetString("input_blob");
		const char *out_blob = cmdLine.GetString("output_blob");
		const char *out_cvg = cmdLine.GetString("output_cvg");
		const char *out_bbox = cmdLine.GetString("output_bbox");
		const char *class_labels = cmdLine.GetString("class_labels");

		if (!input)
			input = DETECTNET_DEFAULT_INPUT;

		if (!out_blob)
		{
			if (!out_cvg)
				out_cvg = DETECTNET_DEFAULT_COVERAGE;
			if (!out_bbox)
				out_bbox = DETECTNET_DEFAULT_BBOX;
		}

		if (!class_labels)
			class_labels = cmdLine.GetString("labels");

		float meanPixel = cmdLine.GetFloat("mean_pixel");

		net = detectNet::Creat