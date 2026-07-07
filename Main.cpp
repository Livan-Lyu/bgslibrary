#include <iostream>

// use to debug
#include <iostream>
#include <opencv2/opencv.hpp>
#include <execinfo.h>

#include "utils/GenericKeys.h"
#include "VideoAnalysis.h"

namespace bgslibrary
{
  class Main
  {
  private:
    Main();

  public:
    static void start(int argc, const char **argv)
    {
      std::cout << "---------------------------------------------" << std::endl;
      std::cout << "ViBe Background Subtraction                  " << std::endl;
      std::cout << "FPGA-accelerated ViBe algorithm              " << std::endl;
      std::cout << "---------------------------------------------" << std::endl;
      std::cout << "Using OpenCV version " << CV_VERSION << std::endl;

      try
      {
        auto key = KEY_ESC;

        do
        {
          auto videoAnalysis = std::make_unique<VideoAnalysis>();

          if (videoAnalysis->setup(argc, argv))
          {
            videoAnalysis->start();

            std::cout << "Processing finished, enter:" << std::endl;
            std::cout << "R - Repeat" << std::endl;
            std::cout << "Q - Quit" << std::endl;

            key = cv::waitKey();
          }

          cv::destroyAllWindows();

        } while (key == KEY_REPEAT);
      }
      catch (const std::exception& ex)
      {
        std::cout << "std::exception:" << ex.what() << std::endl;
        return;
      }
      catch (...)
      {
        std::cout << "Unknow error" << std::endl;
        return;
      }

#ifdef WIN32
      //system("pause");
#endif
    }
  };
}

void print_trace() {
    void* array[10];
    size_t size = backtrace(array, 10);
    char** strings = backtrace_symbols(array, size);
    std::cerr << "--- 崩溃调用栈位置 ---" << std::endl;
    for (size_t i = 0; i < size; i++) {
        std::cerr << strings[i] << std::endl;
    }
    free(strings);
}

int main(int argc, const char **argv)
{
  try {
  bgslibrary::Main::start(argc, argv);
  } 
    catch (const cv::Exception& e) {
        std::cerr << "捕获到 OpenCV 异常: " << e.what() << std::endl;
        print_trace(); // 强制在这里打印是谁一路调用过来的
    }
    catch (const std::exception& e) {
        std::cerr << "捕获到标准异常: " << e.what() << std::endl;
        print_trace();
    }
  return 0;
}
