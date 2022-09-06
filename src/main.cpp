// waifu2x implemented with ncnn library

#include <stdio.h>
#include <algorithm>
#include <queue>
#include <vector>
#include <clocale>
#include <filesystem>
#include <map>
#include <regex>

#if _WIN32
// image decoder and encoder with wic
#include "wic_image.h"
#else // _WIN32
// image decoder and encoder with stb
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_PSD
#define STBI_NO_TGA
#define STBI_NO_GIF
#define STBI_NO_HDR
#define STBI_NO_PIC
#define STBI_NO_STDIO
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#endif // _WIN32
#include "webp_image.h"
#include "opencv2/opencv.hpp"
#include <ncurses.h>
#include <pthread.h>
#if _WIN32
#include <wchar.h>
static wchar_t* optarg = NULL;
static int optind = 1;
static wchar_t getopt(int argc, wchar_t* const argv[], const wchar_t* optstring)
{
    if (optind >= argc || argv[optind][0] != L'-')
        return -1;

    wchar_t opt = argv[optind][1];
    const wchar_t* p = wcschr(optstring, opt);
    if (p == NULL)
        return L'?';

    optarg = NULL;

    if (p[1] == L':')
    {
        optind++;
        if (optind >= argc)
            return L'?';

        optarg = argv[optind];
    }

    optind++;

    return opt;
}

static std::vector<int> parse_optarg_int_array(const wchar_t* optarg)
{
    std::vector<int> array;
    array.push_back(_wtoi(optarg));

    const wchar_t* p = wcschr(optarg, L',');
    while (p)
    {
        p++;
        array.push_back(_wtoi(p));
        p = wcschr(p, L',');
    }

    return array;
}
#else // _WIN32
#include <unistd.h> // getopt()

static std::vector<int> parse_optarg_int_array(const char* optarg)
{
    std::vector<int> array;
    array.push_back(atoi(optarg));

    const char* p = strchr(optarg, ',');
    while (p)
    {
        p++;
        array.push_back(atoi(p));
        p = strchr(p, ',');
    }

    return array;
}
#endif // _WIN32

// ncnn
#include "cpu.h"
#include "gpu.h"
#include "platform.h"

#include "waifu2x.h"

#include "filesystem_utils.h"

static void print_usage()
{
    fprintf(stdout, "Usage: waifu2x-ncnn-vulkan -i infile -o outfile [options]...\n\n");
    fprintf(stdout, "  -h                   show this help\n");
    fprintf(stdout, "  -v                   verbose output\n");
    fprintf(stdout, "  -i input-path        input image path (jpg/png/webp) or directory\n");
    fprintf(stdout, "  -o output-path       output image path (jpg/png/webp) or directory\n");
    fprintf(stdout, "  -n noise-level       denoise level (-1/0/1/2/3, default=0)\n");
    fprintf(stdout, "  -s scale             upscale ratio (1/2/4/8/16/32, default=2)\n");
    fprintf(stdout, "  -t tile-size         tile size (>=32/0=auto, default=0) can be 0,0,0 for multi-gpu\n");
    fprintf(stdout, "  -m model-path        waifu2x model path (default=models-cunet)\n");
    fprintf(stdout, "  -g gpu-id            gpu device to use (-1=cpu, default=auto) can be 0,1,2 for multi-gpu\n");
    fprintf(stdout, "  -j load:proc:save    thread count for load/proc/save (default=1:2:2) can be 1:2,2,2:2 for multi-gpu\n");
    fprintf(stdout, "  -x                   enable tta mode\n");
    fprintf(stdout, "  -f format            output image format (jpg/png/webp, default=ext/png)\n");
}

namespace Console {
#include  <stdio.h>

    int consoleStatus = 0;
    FILE *consoleTTY = NULL;
    SCREEN *screen = NULL;
    
    int init() {
        consoleTTY = fopen("/dev/tty", "r+");
        screen = newterm(NULL, consoleTTY, consoleTTY);
        set_term(screen);
        consoleStatus = 1;
        return 1;
    }
    int end() {
        return endwin();
    }
    int printToCoord(int x, int y, char *console_output)
    {
        if(consoleStatus == 0)
            Console::init();
        
        mvprintw(x, y, console_output);
        refresh();
        return 0;
        /*
        int argc, char ** argv) {

      FILE *f = fopen("/dev/tty", "r+");
      SCREEN *screen = newterm(NULL, f, f);
      set_term(screen);

      //this goes to stdout
      fprintf(stdout, "hello\n");
      //this goes to the console
      fprintf(stderr, "some error\n");
      //this goes to display
      mvprintw(0, 0, "hello ncurses");
      refresh();
      getch();
      endwin();

      return 0;
    }
    */
    }
};

namespace VideoUtilities
{

    //path_t inputFile;
    path_t outputPath;
    path_t output_path;
    std::vector<path_t> filesToEncode;
    int frames_per_second;
    double frameCount = 0;
    int height;
    int width;
    cv::VideoCapture inputVideo;
    
    double getFrameCount()
    {
        return frameCount;
    }
    
    std::vector<path_t> getFilesToEncode()
    {
        return filesToEncode;   
    }
    
    void setOutputTempPath(path_t setOutputPath)
    {
        outputPath = setOutputPath;
        return;
    }
    
    void init(path_t inputFile)
    {
        inputVideo = cv::VideoCapture(inputFile);
        width  = inputVideo.get(cv::CAP_PROP_FRAME_WIDTH);
        height = inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT);  
        frames_per_second = inputVideo.get(cv::CAP_PROP_FPS);
        frameCount = inputVideo.get(cv::CAP_PROP_FRAME_COUNT);
    }
    
    void convertVideo2PNG()
    {
        path_t outputPathDir = outputPath + PATHSTR("/tmp/");
        std::filesystem::create_directory(outputPathDir);
        
        
        cv::Mat image;
        

        //std::cout << "Frames Per Second: " << frames_per_second << std::endl << "Height: " << height << std::endl << "Width: " << width;
        //getchar();
        
        int i = 0;
        //int zero_padding = std::to_string(getFrameCount()).length();
            
        while (inputVideo.isOpened())
        {
            std::string name = std::to_string(i++);
            // Pad with zeroes
            //name = std::string(zero_padding - std::min(zero_padding, name.length()), '0') + name;
            
            // Read next frame in video file
            inputVideo.read(image);

            if (image.empty())
            {
                break;
            }
            // Save image to disk.
            std::string outputFile = outputPathDir + name + ".png";

            filesToEncode.push_back(outputFile);
            
            if(std::filesystem::exists(outputFile))
                printf("file exists: %s\n", outputFile.c_str());//std::cout << "File already exists: " << outputFile << std::endl;
            else {
                cv::imwrite(outputFile, image);
                printf("Succesfully %s to disk\n", outputFile.c_str());//std::cout << "Succesfully saved frame " << outputFile << " to disk." << std::endl;
            }
            
            
        }
        
        return;
    }
    void eraseSubStr(std::string & mainStr, const std::string & toErase)
    {
        // Search for the substring in string
        size_t pos = mainStr.find(toErase);
        if (pos != std::string::npos)
        {
            // If found then erase it from string
            mainStr.erase(pos, toErase.length());
        }
    }
    void convertPNG2Video()
    {
        getchar();
        //std::cout << "Built with OpenCV " << CV_VERSION << endl;
        cv::Mat image;
        //cv::Mat src;
        //cv::VideoCapture capture;
        //capture.open(2);
        //capture >> src;
        bool isColor = true;//(src.type() == CV_8UC3);
        cv::VideoWriter writer;
        int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');  
        double fps = 60.0;
        std::string filename = "live.mp4";
        
        std::cout << "Started writing video... " << std::endl;
        
        //capture >> image;
        cv::Mat tmp = cv::imread(filesToEncode.front());
        cv::Size sizeFrame = cv::Size(tmp.rows, tmp.cols);//image.size();
        
        writer.open(filename, codec, fps, sizeFrame, isColor);
 
        
        int frameCount = VideoUtilities::getFrameCount();
        
        for( auto imageFilePath : filesToEncode)
        {
            eraseSubStr(imageFilePath, "/tmp");
            std::cout << "Writing " << imageFilePath << " to " << filename << std::endl;
            writer.write(cv::imread(imageFilePath));
        }
        /*for (int i = 0 ; i < frameCount ; i ++)
        {
            capture >> image;
            cv::Mat xframe;
            //cv::resize(image,xframe,sizeFrame);
            //writer.write(xframe);
            writer.write(image);
            // imshow("Sample", image);
            // char c = (char)waitKey(1);
            // if(c == 27) break;
        }*/
        std::cout << "Write complete!" << std::endl;
        //capture.release();
        writer.release();
        return; 
    }
    
};

namespace TaskOverview { 
	bool running = 0;
    
    double numberOfJobs = 0;
    double jobsCompleted = 0;
    
    pthread_t updateCompletion;

    void *printCompletion( void* )
    {
        //char *message;
        //message = (char *) ptr;
        char message[512];
        while(running) {
            
            sprintf(message, "Percent completed: %s \n", std::to_string(((jobsCompleted/numberOfJobs)*100.0)).c_str());
            Console::printToCoord(60, 0, message);
            //usleep(2000);
            sleep(1000);
        }
        return NULL;
    }
    
    void initThread() 
    {
        running = true;
        pthread_create( &updateCompletion, NULL, TaskOverview::printCompletion, (void*) NULL);
    }
    void joinThread()
    {
        pthread_join( updateCompletion, NULL);
    }
    void setNumberOfJobs(double njobs)
    {
        numberOfJobs = njobs;
    }
    
    void updateJobsCompleted(double nJobsCompleted)
    {
        jobsCompleted = nJobsCompleted;
    }
    void incrementJobsCompleted()
    {
        jobsCompleted++;   
    }
    //std::map<std::string, float> taskStatus;
	
    //void updateTask(std::string filename, float percentCompleted) { taskStatus.insert(std::make_pair(filename, percentCompleted)); }
        //void printTaskStatus(){};
		//const TaskManager getInstance(){ static TaskManager instance; return instance; } 
};
namespace TaskManager {
    std::map<std::string, float> taskStatus;
    //void printTaskStatus(){};
    void updateTask(std::string filename, float percentCompleted) { taskStatus.insert(std::make_pair(filename, percentCompleted)); }
};


class Task
{
public:
    int id;
    int webp;
    int scale;

    path_t inpath;
    path_t outpath;

    ncnn::Mat inimage;
    ncnn::Mat outimage;
};

class TaskQueue
{
public:
    TaskQueue()
    {
    }

    void put(const Task& v)
    {
        lock.lock();

        while (tasks.size() >= 8) // FIXME hardcode queue length
        {
            condition.wait(lock);
        }

        tasks.push(v);

        lock.unlock();

        condition.signal();
    }

    void get(Task& v)
    {
        lock.lock();

        while (tasks.size() == 0)
        {
            condition.wait(lock);
        }

        v = tasks.front();
        tasks.pop();

        lock.unlock();

        condition.signal();
    }

private:
    ncnn::Mutex lock;
    ncnn::ConditionVariable condition;
    std::queue<Task> tasks;
};

TaskQueue toproc;
TaskQueue tosave;

class LoadThreadParams
{
public:
    int scale;
    int jobs_load;

    // session data
    std::vector<path_t> input_files;
    std::vector<path_t> output_files;
    
};

void* load(void* args)
{
    const LoadThreadParams* ltp = (const LoadThreadParams*)args;
    const int count = ltp->input_files.size();
    const int scale = ltp->scale;

    #pragma omp parallel for schedule(static,1) num_threads(ltp->jobs_load)
    for (int i=0; i<count; i++)
    {
        const path_t& imagepath = ltp->input_files[i];

        int webp = 0;

        unsigned char* pixeldata = 0;
        int w;
        int h;
        int c;

#if _WIN32
        FILE* fp = _wfopen(imagepath.c_str(), L"rb");
#else
        FILE* fp = fopen(imagepath.c_str(), "rb");
#endif
        if (fp)
        {
            // read whole file
            unsigned char* filedata = 0;
            int length = 0;
            {
                fseek(fp, 0, SEEK_END);
                length = ftell(fp);
                rewind(fp);
                filedata = (unsigned char*)malloc(length);
                if (filedata)
                {
                    fread(filedata, 1, length, fp);
                }
                fclose(fp);
            }

            if (filedata)
            {
                pixeldata = webp_load(filedata, length, &w, &h, &c);
                if (pixeldata)
                {
                    webp = 1;
                }
                else
                {
                    // not webp, try jpg png etc.
#if _WIN32
                    pixeldata = wic_decode_image(imagepath.c_str(), &w, &h, &c);
#else // _WIN32
                    pixeldata = stbi_load_from_memory(filedata, length, &w, &h, &c, 0);
                    if (pixeldata)
                    {
                        // stb_image auto channel
                        if (c == 1)
                        {
                            // grayscale -> rgb
                            stbi_image_free(pixeldata);
                            pixeldata = stbi_load_from_memory(filedata, length, &w, &h, &c, 3);
                            c = 3;
                        }
                        else if (c == 2)
                        {
                            // grayscale + alpha -> rgba
                            stbi_image_free(pixeldata);
                            pixeldata = stbi_load_from_memory(filedata, length, &w, &h, &c, 4);
                            c = 4;
                        }
                    }
#endif // _WIN32
                }

                free(filedata);
            }
        }
        if (pixeldata)
        {
            Task v;
            v.id = i;
            v.webp = webp;
            v.scale = scale;
            v.inpath = imagepath;
            v.outpath = ltp->output_files[i];

            v.inimage = ncnn::Mat(w, h, (void*)pixeldata, (size_t)c, c);

            path_t ext = get_file_extension(v.outpath);
            if (c == 4 && (ext == PATHSTR("jpg") || ext == PATHSTR("JPG") || ext == PATHSTR("jpeg") || ext == PATHSTR("JPEG")))
            {
                path_t output_filename2 = ltp->output_files[i] + PATHSTR(".png");
                v.outpath = output_filename2;
#if _WIN32
                fwprintf(stderr, L"image %ls has alpha channel ! %ls will output %ls\n", imagepath.c_str(), imagepath.c_str(), output_filename2.c_str());
#else // _WIN32
                fprintf(stderr, "image %s has alpha channel ! %s will output %s\n", imagepath.c_str(), imagepath.c_str(), output_filename2.c_str());
#endif // _WIN32
            }

            toproc.put(v);
        }
        else
        {
#if _WIN32
            fwprintf(stderr, L"decode image %ls failed\n", imagepath.c_str());
#else // _WIN32
            fprintf(stderr, "decode image %s failed\n", imagepath.c_str());
#endif // _WIN32
        }
    }

    return 0;
}

class ProcThreadParams
{
public:
    const Waifu2x* waifu2x;
    int verbose;
};

void* proc(void* args)
{
    const ProcThreadParams* ptp = (const ProcThreadParams*)args;
    const Waifu2x* waifu2x = ptp->waifu2x;
    const bool verbose = ptp->verbose;
    for (;;)
    {
        Task v;

        toproc.get(v);

        if (v.id == -233)
            break;

        const int scale = v.scale;
        if (scale == 1)
        {
            v.outimage = ncnn::Mat(v.inimage.w, v.inimage.h, (size_t)v.inimage.elemsize, (int)v.inimage.elemsize);
            waifu2x->process(v.inimage, v.outimage, std::string(v.inpath));

            tosave.put(v);
            continue;
        }

        int scale_run_count = 0;
        if (scale == 2)
        {
            scale_run_count = 1;
        }
        if (scale == 4)
        {
            scale_run_count = 2;
        }
        if (scale == 8)
        {
            scale_run_count = 3;
        }
        if (scale == 16)
        {
            scale_run_count = 4;
        }
        if (scale == 32)
        {
            scale_run_count = 5;
        }

        v.outimage = ncnn::Mat(v.inimage.w * 2, v.inimage.h * 2, (size_t)v.inimage.elemsize, (int)v.inimage.elemsize);
        waifu2x->process(v.inimage, v.outimage, std::string(v.inpath));

        for (int i = 1; i < scale_run_count; i++)
        {
            ncnn::Mat tmp = v.outimage;
            v.outimage = ncnn::Mat(tmp.w * 2, tmp.h * 2, (size_t)v.inimage.elemsize, (int)v.inimage.elemsize);
            waifu2x->process(tmp, v.outimage, std::string(v.inpath));
        }

        tosave.put(v);
    }

    return 0;
}

class SaveThreadParams
{
public:
    int verbose;
};

void* save(void* args)
{
    const SaveThreadParams* stp = (const SaveThreadParams*)args;
    const int verbose = stp->verbose;

    for (;;)
    {
        Task v;

        tosave.get(v);

        if (v.id == -233)
            break;

        // free input pixel data
        {
            unsigned char* pixeldata = (unsigned char*)v.inimage.data;
            if (v.webp == 1)
            {
                free(pixeldata);
            }
            else
            {
#if _WIN32
                free(pixeldata);
#else
                stbi_image_free(pixeldata);
#endif
            }
        }

        int success = 0;

        path_t ext = get_file_extension(v.outpath);

        if (ext == PATHSTR("webp") || ext == PATHSTR("WEBP"))
        {
            success = webp_save(v.outpath.c_str(), v.outimage.w, v.outimage.h, v.outimage.elempack, (const unsigned char*)v.outimage.data);
        }
        else if (ext == PATHSTR("png") || ext == PATHSTR("PNG"))
        {
#if _WIN32
            success = wic_encode_image(v.outpath.c_str(), v.outimage.w, v.outimage.h, v.outimage.elempack, v.outimage.data);
#else
            success = stbi_write_png(v.outpath.c_str(), v.outimage.w, v.outimage.h, v.outimage.elempack, v.outimage.data, 0);
#endif
        }
        else if (ext == PATHSTR("jpg") || ext == PATHSTR("JPG") || ext == PATHSTR("jpeg") || ext == PATHSTR("JPEG"))
        {
#if _WIN32
            success = wic_encode_jpeg_image(v.outpath.c_str(), v.outimage.w, v.outimage.h, v.outimage.elempack, v.outimage.data);
#else
            success = stbi_write_jpg(v.outpath.c_str(), v.outimage.w, v.outimage.h, v.outimage.elempack, v.outimage.data, 100);
#endif
        }
        if (success)
        {
            if (verbose)
            {
#if _WIN32
                fwprintf(stdout, L"%ls -> %ls done\n", v.inpath.c_str(), v.outpath.c_str());
                TaskOverview::incrementJobsCompleted();
                //char output[256];
                //fwprintf(output, L"%ls -> %ls done\n", v.inpath.c_str(), v.outpath.c_str());
                //Console::printToCoord(0, 0, output);
#else
                TaskOverview::incrementJobsCompleted();
                //char output[256];
                //sprintf(output, "%s -> %s done\n", v.inpath.c_str(), v.outpath.c_str());
                //Console::printToCoord(0, 0, output);
                fprintf(stdout, "%s -> %s done\n", v.inpath.c_str(), v.outpath.c_str());
                //todo VideoTaskManager::deleteFile(v.inp
#endif
            }
        }
        else
        {
#if _WIN32
            fwprintf(stderr, L"encode image %ls failed\n", v.outpath.c_str());
#else
            fprintf(stderr, "encode image %s failed\n", v.outpath.c_str());
#endif
        }
    }

    return 0;
}





int main(int argc, char** argv)
{

    path_t inputpath;
    path_t outputpath;
    int noise = 0;
    int scale = 2;
    std::vector<int> tilesize;
    path_t model = PATHSTR("models-cunet");
    std::vector<int> gpuid;
    int jobs_load = 1;
    std::vector<int> jobs_proc;
    int jobs_save = 2;
    int verbose = 0;
    int tta_mode = 0;
    path_t format = PATHSTR("png");
        
    int opt;
    while ((opt = getopt(argc, argv, "i:o:n:s:t:m:g:j:f:vxh")) != -1)
    {
        switch (opt)
        {
        case 'i':
            inputpath = optarg;
            break;
        case 'o':
            outputpath = optarg;
            break;
        case 'n':
            noise = atoi(optarg);
            break;
        case 's':
            scale = atoi(optarg);
            break;
        case 't':
            tilesize = parse_optarg_int_array(optarg);
            break;
        case 'm':
            model = optarg;
            break;
        case 'g':
            gpuid = parse_optarg_int_array(optarg);
            break;
        case 'j':
            sscanf(optarg, "%d:%*[^:]:%d", &jobs_load, &jobs_save);
            jobs_proc = parse_optarg_int_array(strchr(optarg, ':') + 1);
            break;
        case 'f':
            format = optarg;
            break;
        case 'v':
            verbose = 1;
            break;
        case 'x':
            tta_mode = 1;
            break;
        case 'h':
        default:
            print_usage();
            return -1;
        }
    }


    if (inputpath.empty() || outputpath.empty())
    {
        print_usage();
        return -1;
    }

    if (noise < -1 || noise > 3)
    {
        fprintf(stderr, "invalid noise argument\n");
        return -1;
    }

    if (!(scale == 1 || scale == 2 || scale == 4 || scale == 8 || scale == 16 || scale == 32))
    {
        fprintf(stderr, "invalid scale argument\n");
        return -1;
    }

    if (tilesize.size() != (gpuid.empty() ? 1 : gpuid.size()) && !tilesize.empty())
    {
        fprintf(stderr, "invalid tilesize argument\n");
        return -1;
    }

    for (int i=0; i<(int)tilesize.size(); i++)
    {
        if (tilesize[i] != 0 && tilesize[i] < 32)
        {
            fprintf(stderr, "invalid tilesize argument\n");
            return -1;
        }
    }

    if (jobs_load < 1 || jobs_save < 1)
    {
        fprintf(stderr, "invalid thread count argument\n");
        return -1;
    }

    if (jobs_proc.size() != (gpuid.empty() ? 1 : gpuid.size()) && !jobs_proc.empty())
    {
        fprintf(stderr, "invalid jobs_proc thread count argument\n");
        return -1;
    }

    for (int i=0; i<(int)jobs_proc.size(); i++)
    {
        if (jobs_proc[i] < 1)
        {
            fprintf(stderr, "invalid jobs_proc thread count argument\n");
            return -1;
        }
    }

    if (!path_is_directory(outputpath))
    {
        // guess format from outputpath no matter what format argument specified
        path_t ext = get_file_extension(outputpath);

        if (ext == PATHSTR("png") || ext == PATHSTR("PNG"))
        {
            format = PATHSTR("png");
        }
        else if (ext == PATHSTR("webp") || ext == PATHSTR("WEBP"))
        {
            format = PATHSTR("webp");
        }
        else if (ext == PATHSTR("jpg") || ext == PATHSTR("JPG") || ext == PATHSTR("jpeg") || ext == PATHSTR("JPEG"))
        {
            format = PATHSTR("jpg");
        }
        else
        {
            fprintf(stderr, "invalid outputpath extension type\n");
            return -1;
        }
    }

    if (format != PATHSTR("png") && format != PATHSTR("webp") && format != PATHSTR("jpg"))
    {
        fprintf(stderr, "invalid format argument\n");
        return -1;
    }

    
    //if(!std::filesystem::exists(inputpath)) {
    ///    printf("file doesn't exists: %s\n", inputpath.c_str());
    //    return -1;
    //}
    path_t outputPathDir = outputpath;

    VideoUtilities::setOutputTempPath(outputPathDir);
    VideoUtilities::init(inputpath);
    VideoUtilities::convertVideo2PNG();
    
    //Console::init();
    //char* taskText = "Starting TaskOverview Thread\n";
    //Console::printToCoord(0, 0, taskText);
    //getchar();
    
    //TaskOverview::setNumberOfJobs(VideoUtilities::getFrameCount());
    
    //TaskOverview::initThread();
    //TaskOverview::joinThread();
    
    inputpath = outputPathDir + PATHSTR("/tmp");
    
    // collect input and output filepath
    std::vector<path_t> input_files;
    std::vector<path_t> output_files;
    {
        if (path_is_directory(inputpath) && path_is_directory(outputpath))
        {
            std::vector<path_t> filenames;
            int lr = list_directory(inputpath, filenames);
            if (lr != 0)
                return -1;

            const int count = filenames.size();
            input_files.resize(count);
            output_files.resize(count);

            path_t last_filename;
            path_t last_filename_noext;
            for (int i=0, j=0; i<count; i++)
            {
                path_t filename = filenames[i];
                path_t filename_noext = get_file_name_without_extension(filename);
                path_t output_filename = filename_noext + PATHSTR('.') + format;

				path_t output_path = outputpath + PATHSTR('/') + output_filename;
				if(std::filesystem::exists(output_path))
				  {
					printf("file exists: %s\n", output_path.c_str());
                    //TaskOverview::incrementJobsCompleted();
					
				}
				else {
					// filename list is sorted, check if output image path conflicts
					if (filename_noext == last_filename_noext)
					{
						path_t output_filename2 = filename + PATHSTR('.') + format;
	#if _WIN32
						fwprintf(stderr, L"both %ls and %ls output %ls ! %ls will output %ls\n", filename.c_str(), last_filename.c_str(), output_filename.c_str(), filename.c_str(), output_filename2.c_str());
	#else
						fprintf(stderr, "both %s and %s output %s ! %s will output %s\n", filename.c_str(), last_filename.c_str(), output_filename.c_str(), filename.c_str(), output_filename2.c_str());
	#endif
						output_filename = output_filename2;
					}
					else
					{
						last_filename = filename;
						last_filename_noext = filename_noext;
					}

					input_files[j] = inputpath + PATHSTR('/') + filename;
					output_files[j] = outputpath + PATHSTR('/') + output_filename;
					j++;
				}
            }
        }
        else if (!path_is_directory(inputpath) && !path_is_directory(outputpath))
        {
            input_files.push_back(inputpath);
            output_files.push_back(outputpath);
        }
        else
        {
            fprintf(stderr, "inputpath and outputpath must be either file or directory at the same time\n");
            return -1;
        }
    }
    //std::sort(input_files.begin(),  input_files.end,  [](std::string a, std::string b) { return (std::abs(a) < std::abs(b));}
    //std::sort(output_files.begin(), output_files.end, [](std::string a, std::string b) { return (std::abs(a) < std::abs(b));}
              
    int prepadding = 0;

    if (model.find(PATHSTR("models-cunet")) != path_t::npos)
    {
        if (noise == -1)
        {
            prepadding = 18;
        }
        else if (scale == 1)
        {
            prepadding = 28;
        }
        else if (scale == 2 || scale == 4 || scale == 8 || scale == 16 || scale == 32)
        {
            prepadding = 18;
        }
    }
    else if (model.find(PATHSTR("models-upconv_7_anime_style_art_rgb")) != path_t::npos)
    {
        prepadding = 7;
    }
    else if (model.find(PATHSTR("models-upconv_7_photo")) != path_t::npos)
    {
        prepadding = 7;
    }
    else
    {
        fprintf(stderr, "unknown model dir type\n");
        return -1;
    }

#if _WIN32
    wchar_t parampath[256];
    wchar_t modelpath[256];
    if (noise == -1)
    {
        swprintf(parampath, 256, L"%s/scale2.0x_model.param", model.c_str());
        swprintf(modelpath, 256, L"%s/scale2.0x_model.bin", model.c_str());
    }
    else if (scale == 1)
    {
        swprintf(parampath, 256, L"%s/noise%d_model.param", model.c_str(), noise);
        swprintf(modelpath, 256, L"%s/noise%d_model.bin", model.c_str(), noise);
    }
    else if (scale == 2 || scale == 4 || scale == 8 || scale == 16 || scale == 32)
    {
        swprintf(parampath, 256, L"%s/noise%d_scale2.0x_model.param", model.c_str(), noise);
        swprintf(modelpath, 256, L"%s/noise%d_scale2.0x_model.bin", model.c_str(), noise);
    }
#else
    char parampath[256];
    char modelpath[256];
    if (noise == -1)
    {
        sprintf(parampath, "%s/scale2.0x_model.param", model.c_str());
        sprintf(modelpath, "%s/scale2.0x_model.bin", model.c_str());
    }
    else if (scale == 1)
    {
        sprintf(parampath, "%s/noise%d_model.param", model.c_str(), noise);
        sprintf(modelpath, "%s/noise%d_model.bin", model.c_str(), noise);
    }
    else if (scale == 2 || scale == 4 || scale == 8 || scale == 16 || scale == 32)
    {
        sprintf(parampath, "%s/noise%d_scale2.0x_model.param", model.c_str(), noise);
        sprintf(modelpath, "%s/noise%d_scale2.0x_model.bin", model.c_str(), noise);
    }
#endif

    path_t paramfullpath = sanitize_filepath(parampath);
    path_t modelfullpath = sanitize_filepath(modelpath);

#if _WIN32
    CoInitializeEx(NULL, COINIT_MULTITHREADED);
#endif

    ncnn::create_gpu_instance();

    if (gpuid.empty())
    {
        gpuid.push_back(ncnn::get_default_gpu_index());
    }

    const int use_gpu_count = (int)gpuid.size();

    if (jobs_proc.empty())
    {
        jobs_proc.resize(use_gpu_count, 2);
    }

    if (tilesize.empty())
    {
        tilesize.resize(use_gpu_count, 0);
    }

    int cpu_count = std::max(1, ncnn::get_cpu_count());
    jobs_load = std::min(jobs_load, cpu_count);
    jobs_save = std::min(jobs_save, cpu_count);

    int gpu_count = ncnn::get_gpu_count();
    for (int i=0; i<use_gpu_count; i++)
    {
        if (gpuid[i] < -1 || gpuid[i] >= gpu_count)
        {
            fprintf(stderr, "invalid gpu device\n");

            ncnn::destroy_gpu_instance();
            return -1;
        }
    }

    int total_jobs_proc = 0;
    int jobs_proc_per_gpu[16] = {0};
    
    if(verbose == true)
        std::cout << "Gpu count: " << gpu_count << std::endl << "Use gpu count: " << use_gpu_count << std::endl;
    
    for (int i=0; i<use_gpu_count; i++)
    {
        if (gpuid[i] == -1)
        {
            jobs_proc[i] = std::min(jobs_proc[i], cpu_count);
            total_jobs_proc += 1;
        }
        else
        {
            total_jobs_proc += jobs_proc[i];
            jobs_proc_per_gpu[gpuid[i]] += jobs_proc[i];
        }
    }

    for (int i=0; i<use_gpu_count; i++)
    {
        if (tilesize[i] != 0)
            continue;

        if (gpuid[i] == -1)
        {
            // cpu only
            tilesize[i] = 400;
            continue;
        }

        uint32_t heap_budget = ncnn::get_gpu_device(gpuid[i])->get_heap_budget();

        if (path_is_directory(inputpath) && path_is_directory(outputpath))
        {
            // multiple gpu jobs share the same heap
            heap_budget /= jobs_proc_per_gpu[gpuid[i]];
        }

        // more fine-grained tilesize policy here
        if (model.find(PATHSTR("models-cunet")) != path_t::npos)
        {
            if (heap_budget > 2600)
                tilesize[i] = 400;
            else if (heap_budget > 740)
                tilesize[i] = 200;
            else if (heap_budget > 250)
                tilesize[i] = 100;
            else
                tilesize[i] = 32;
        }
        else if (model.find(PATHSTR("models-upconv_7_anime_style_art_rgb")) != path_t::npos
            || model.find(PATHSTR("models-upconv_7_photo")) != path_t::npos)
        {
            if (heap_budget > 1900)
                tilesize[i] = 400;
            else if (heap_budget > 550)
                tilesize[i] = 200;
            else if (heap_budget > 190)
                tilesize[i] = 100;
            else
                tilesize[i] = 32;
        }
    }

    {
        std::vector<Waifu2x*> waifu2x(use_gpu_count);

        for (int i=0; i<use_gpu_count; i++)
        {
            int num_threads = gpuid[i] == -1 ? jobs_proc[i] : 1;

            waifu2x[i] = new Waifu2x(gpuid[i], tta_mode, num_threads);

            waifu2x[i]->load(paramfullpath, modelfullpath);

            waifu2x[i]->noise = noise;
            waifu2x[i]->scale = (scale >= 2) ? 2 : scale;
            waifu2x[i]->tilesize = tilesize[i];
            waifu2x[i]->prepadding = prepadding;
        }

        // main routine
        {
            // load image
            LoadThreadParams ltp;
            ltp.scale = scale;
            ltp.jobs_load = jobs_load;
            ltp.input_files = input_files;
            ltp.output_files = output_files;

            ncnn::Thread load_thread(load, (void*)&ltp);

            // waifu2x proc
            std::vector<ProcThreadParams> ptp(use_gpu_count);
            for (int i=0; i<use_gpu_count; i++)
            {
                ptp[i].waifu2x = waifu2x[i];
            }

            std::vector<ncnn::Thread*> proc_threads(total_jobs_proc);
            {
                int total_jobs_proc_id = 0;
                for (int i=0; i<use_gpu_count; i++)
                {
                    if (gpuid[i] == -1)
                    {
                        proc_threads[total_jobs_proc_id++] = new ncnn::Thread(proc, (void*)&ptp[i]);
                    }
                    else
                    {
                        for (int j=0; j<jobs_proc[i]; j++)
                        {
                            proc_threads[total_jobs_proc_id++] = new ncnn::Thread(proc, (void*)&ptp[i]);
                        }
                    }
                }
            }

            // save image
            SaveThreadParams stp;
            stp.verbose = verbose;

            std::vector<ncnn::Thread*> save_threads(jobs_save);
            for (int i=0; i<jobs_save; i++)
            {
                save_threads[i] = new ncnn::Thread(save, (void*)&stp);
            }

            // end
            load_thread.join();

            Task end;
            end.id = -233;

            for (int i=0; i<total_jobs_proc; i++)
            {
                toproc.put(end);
            }

            for (int i=0; i<total_jobs_proc; i++)
            {
                proc_threads[i]->join();
                delete proc_threads[i];
            }

            for (int i=0; i<jobs_save; i++)
            {
                tosave.put(end);
            }

            for (int i=0; i<jobs_save; i++)
            {
                save_threads[i]->join();
                delete save_threads[i];
            }
        }

        for (int i=0; i<use_gpu_count; i++)
        {
            delete waifu2x[i];
        }
        waifu2x.clear();
    }

    ncnn::destroy_gpu_instance();
    
    //VideoUtilities::convertPNG2Video();
    
    return 0;
}
