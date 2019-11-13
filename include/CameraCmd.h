#ifndef CAMERACMD_H
#define CAMERACMD_H

enum ImageType {
    DEPTH = 1,
    COLOR = 2,
    COLOR_DEPTH = COLOR | DEPTH,
};

enum NetCamCmd {
	CaptureImage = 0,
	GetCameraIntri = 11,
	GetCameraStatus = 19,
};

constexpr int Encode32FBias = 32768;

#endif // CAMERACMD_H
