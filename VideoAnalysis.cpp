#include "VideoAnalysis.h"

namespace bgslibrary
{
  VideoAnalysis::VideoAnalysis() :
    use_file(false), use_camera(false), cameraIndex(0),
    frameToStop(0)
  {
    debug_construction(VideoAnalysis);
  }

  VideoAnalysis::~VideoAnalysis() {
    debug_destruction(VideoAnalysis);
  }

  bool VideoAnalysis::setup(int argc, const char **argv)
  {
    bool flag = false;

#if CV_MAJOR_VERSION == 2
    const char* keys =
      "{hp|help|false|Print this message}"
      "{uf|use_file|false|Use video file}"
      "{fn|filename||Specify video file}"
      "{uc|use_cam|false|Use camera}"
      "{ca|camera|0|Specify camera index}"
      "{st|stopAt|0|Frame number to stop}"
      ;
#elif CV_MAJOR_VERSION >= 3
    const std::string keys =
      "{h help ?     |     | Print this message   }"
      "{uf use_file  |false| Use a video file     }"
      "{fn filename  |     | Specify a video file }"
      "{uc use_cam   |false| Use a webcamera      }"
      "{ca camera    | 0   | Specify camera index }"
      "{st stopAt    | 0   | Frame number to stop }"
      ;
#endif

    cv::CommandLineParser cmd(argc, argv, keys);

#if CV_MAJOR_VERSION == 2
    if (argc <= 1 || cmd.get<bool>("help") == true)
    {
      std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
      std::cout << "Available options:" << std::endl;
      cmd.printParams();
      return false;
    }
#elif CV_MAJOR_VERSION >= 3
    if (argc <= 1 || cmd.has("help"))
    {
      std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
      std::cout << "Available options:" << std::endl;
      cmd.printMessage();
      return false;
    }
    if (!cmd.check())
    {
      cmd.printErrors();
      return false;
    }
#endif

    use_file = cmd.get<bool>("uf");
    filename = cmd.get<std::string>("fn");
    use_camera = cmd.get<bool>("uc");
    cameraIndex = cmd.get<int>("ca");
    frameToStop = cmd.get<int>("st");

    std::cout << "use_file:    " << use_file << std::endl;
    std::cout << "filename:    " << filename << std::endl;
    std::cout << "use_camera:  " << use_camera << std::endl;
    std::cout << "cameraIndex: " << cameraIndex << std::endl;
    std::cout << "frameToStop: " << frameToStop << std::endl;

    if (use_file) {
      if (filename.empty()) {
        std::cout << "Specify filename" << std::endl;
        return false;
      }

      flag = true;
    }

    if (use_camera)
      flag = true;

    return flag;
  }

  void VideoAnalysis::start()
  {
    do {
      videoCapture = std::make_unique<VideoCapture>();
      frameProcessor = std::make_shared<FrameProcessor>();

      videoCapture->setFrameProcessor(frameProcessor);

      if (use_file)
        videoCapture->setVideo(filename);

      if (use_camera)
        videoCapture->setCamera(cameraIndex);

      videoCapture->start();

      frameProcessor->finish();

      if (use_file || use_camera)
        break;

      auto key = cv::waitKey(500);
      if (key == KEY_ESC)
        break;

    } while (1);
  }
}
