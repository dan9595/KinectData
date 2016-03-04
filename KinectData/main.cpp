#include "main.h"

#include <fstream>

#include <Kinect.h>

using namespace std;

// Intermediate Buffers
CameraSpacePoint depth2xyz[width*height];			 // Maps depth pixels to 3d coordinates
CameraSpacePoint BodyJoints[BODY_COUNT][JointType::JointType_Count];
Vector4 BodyJointOrientations[BODY_COUNT][JointType::JointType_Count];
int detectedBodyCount;
int bodyPoints;

// Kinect Variables
IKinectSensor* sensor;             // Kinect sensor
IMultiSourceFrameReader* reader;   // Kinect data source
ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates

bool initKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Body | FrameSourceTypes::FrameSourceTypes_BodyIndex,
			&reader);
		return reader;
	}
	else {
		return false;
	}
}

void getBodyData(IMultiSourceFrame *frame) {
	IBodyFrame *bodyframe;
	IBodyFrameReference *frameref = NULL;
	frame->get_BodyFrameReference(&frameref);
	frameref->AcquireFrame(&bodyframe);
	if (frameref) frameref->Release();
	if (!bodyframe) return;

	IBody *body[BODY_COUNT] = { 0 };
	bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);

	for (int count = 0; count < BODY_COUNT; count++) {
		BOOLEAN tracked = false;
		body[count]->get_IsTracked(&tracked);
		if (tracked) {
			Joint Joint[JointType::JointType_Count];
			JointOrientation JointOrientation[JointType::JointType_Count];
			body[count]->GetJoints(JointType::JointType_Count, Joint);
			body[count]->GetJointOrientations(JointType::JointType_Count, JointOrientation);
			for (int j = 0; j < JointType::JointType_Count; j++) {
				BodyJoints[detectedBodyCount][Joint[j].JointType] = Joint[j].Position;
				BodyJointOrientations[detectedBodyCount][JointOrientation[j].JointType] = JointOrientation[j].Orientation;
			}
			detectedBodyCount++;
		}
	}

	if (bodyframe) bodyframe->Release();
}

void getDepthData(IMultiSourceFrame *frame) {
	//get BodyIndex data
	IBodyIndexFrame *bodyIndexFrame;
	IBodyIndexFrameReference *bIdxframeref = NULL;
	frame->get_BodyIndexFrameReference(&bIdxframeref);
	bIdxframeref->AcquireFrame(&bodyIndexFrame);
	if (bIdxframeref) bIdxframeref->Release();
	if (!bodyIndexFrame) return;

	unsigned int bIdxsz = 0;
	unsigned char *bIdxbuffer = nullptr;
	bodyIndexFrame->AccessUnderlyingBuffer(&bIdxsz, &bIdxbuffer);

	//get Depth data
	IDepthFrame *DepthFrame;
	IDepthFrameReference *dframeref = NULL;
	frame->get_DepthFrameReference(&dframeref);
	dframeref->AcquireFrame(&DepthFrame);
	if (dframeref) dframeref->Release();
	if (!DepthFrame) return;

	unsigned int dsz = 0;
	unsigned short *dbuffer = nullptr;
	DepthFrame->AccessUnderlyingBuffer(&dsz, &dbuffer);

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			unsigned int index = y * width + x;
			if (bIdxbuffer[index] == 0xff) {
				dbuffer[index] = 0;
			}
			else bodyPoints++;
		}
	}

	mapper->MapDepthFrameToCameraSpace(height * width, dbuffer, height*width, depth2xyz);

	if (bodyIndexFrame) bodyIndexFrame->Release();
	if (DepthFrame) DepthFrame->Release();

}

void getKinectData() {
	IMultiSourceFrame* frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		getBodyData(frame);
		getDepthData(frame);
	}
	if (frame) frame->Release();
}
 
int main(int argc, char* argv[]) {
	//if (!init(argc, argv)) return 1;
	if (!initKinect()) return 1;

	getKinectData();

	
	//print
	ofstream out ("data.txt");
	int i = 0;
	for (int j = 0; j < JointType::JointType_Count; j++) {
		out << "Joint num " << j << endl;
		out << /*"position : " <<*/ BodyJoints[i][j].X << " " << BodyJoints[i][j].Y << " " << BodyJoints[i][j].Z << endl;
		out << BodyJointOrientations[i][j].w << " " << BodyJointOrientations[i][j].x << " " << BodyJointOrientations[i][j].y << " " << BodyJointOrientations[i][j].z << endl;
	}

	out << endl;
	out << "number of points : " << bodyPoints << endl;
	for (i = 0; i < width*height; i++) {
		if (depth2xyz[i].Z != 0) {
			out << depth2xyz[i].X << " " << depth2xyz[i].Y << " " << depth2xyz[i].Z << endl;
		}
	}


	return 0;
}
