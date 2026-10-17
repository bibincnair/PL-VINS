#include <boost/algorithm/string.hpp>
#include <filesystem>

#include <iomanip>
#include <iostream>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camodocal/chessboard/Chessboard.h"
#include "camodocal/calib/CameraCalibration.h"
#include "camodocal/gpl/gpl.h"

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv,
        "{help ? | | Print help}"
        "{@input | calibrationdata | Input image directory}"
        "{width w | 8 | Chessboard inner corners in x}"
        "{height h | 12 | Chessboard inner corners in y}"
        "{size s | 7 | Square size in mm}"
        "{prefix p | left- | Image filename prefix}"
        "{file-extension e | .png | Image extension}"
        "{camera-model | mei | kannala-brandt, mei, pinhole or scaramuzza}"
        "{camera-name | camera | Camera name and output prefix}"
        "{opencv | true | Use OpenCV corner detection}"
        "{view-results | false | Display calibration images}"
        "{verbose v | true | Print progress}");
    if (parser.has("help")) { parser.printMessage(); return 0; }
    const cv::Size boardSize(parser.get<int>("width"), parser.get<int>("height"));
    const float squareSize=parser.get<float>("size");
    const auto inputDir=parser.get<std::string>("@input");
    const auto cameraModel=parser.get<std::string>("camera-model");
    const auto cameraName=parser.get<std::string>("camera-name");
    const auto prefix=parser.get<std::string>("prefix");
    const auto fileExtension=parser.get<std::string>("file-extension");
    const bool useOpenCV=parser.get<bool>("opencv"), viewResults=parser.get<bool>("view-results"), verbose=parser.get<bool>("verbose");
    if (!parser.check() || boardSize.width<2 || boardSize.height<2 || squareSize<=0) {
        parser.printErrors(); return 1;
    }
    if (!std::filesystem::is_directory(inputDir)) {
        std::cerr << "Cannot find input directory: " << inputDir << std::endl; return 1;
    }

    camodocal::Camera::ModelType modelType;
    if (boost::iequals(cameraModel, "kannala-brandt"))
    {
        modelType = camodocal::Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(cameraModel, "mei"))
    {
        modelType = camodocal::Camera::MEI;
    }
    else if (boost::iequals(cameraModel, "pinhole"))
    {
        modelType = camodocal::Camera::PINHOLE;
    }
    else if (boost::iequals(cameraModel, "scaramuzza"))
    {
        modelType = camodocal::Camera::SCARAMUZZA;
    }
    else
    {
        std::cerr << "# ERROR: Unknown camera model: " << cameraModel << std::endl;
        return 1;
    }

    switch (modelType)
    {
    case camodocal::Camera::KANNALA_BRANDT:
        std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
        break;
    case camodocal::Camera::MEI:
        std::cout << "# INFO: Camera model: Mei" << std::endl;
        break;
    case camodocal::Camera::PINHOLE:
        std::cout << "# INFO: Camera model: Pinhole" << std::endl;
        break;
    case camodocal::Camera::SCARAMUZZA:
        std::cout << "# INFO: Camera model: Scaramuzza-Omnidirect" << std::endl;
        break;
    }

    // look for images in input directory
    std::vector<std::string> imageFilenames;
    for (std::filesystem::directory_iterator itr(inputDir); itr != std::filesystem::directory_iterator(); ++itr)
    {
        if (!std::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        std::string filename = itr->path().filename().string();

        // check if prefix matches
        if (!prefix.empty())
        {
            if (filename.compare(0, prefix.length(), prefix) != 0)
            {
                continue;
            }
        }

        // check if file extension matches
        if (filename.size()<fileExtension.size() || filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
        {
            continue;
        }

        imageFilenames.push_back(itr->path().string());

        if (verbose)
        {
            std::cerr << "# INFO: Adding " << imageFilenames.back() << std::endl;
        }
    }

    if (imageFilenames.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # images: " << imageFilenames.size() << std::endl;
    }

    std::sort(imageFilenames.begin(), imageFilenames.end());

    cv::Mat image = cv::imread(imageFilenames.front(), -1);
    if (image.empty()) { std::cerr << "Cannot read first image" << std::endl; return 1; }
    const cv::Size frameSize = image.size();

    camodocal::CameraCalibration calibration(modelType, cameraName, frameSize, boardSize, squareSize);
    calibration.setVerbose(verbose);

    std::vector<bool> chessboardFound(imageFilenames.size(), false);
    for (size_t i = 0; i < imageFilenames.size(); ++i)
    {
        image = cv::imread(imageFilenames.at(i), -1);

        if (image.empty() || image.size()!=frameSize) { std::cerr << "Inconsistent calibration image" << std::endl; return 1; }
        camodocal::Chessboard chessboard(boardSize, image);

        chessboard.findCorners(useOpenCV);
        if (chessboard.cornersFound())
        {
            if (verbose)
            {
                std::cerr << "# INFO: Detected chessboard in image " << i + 1 << ", " << imageFilenames.at(i) << std::endl;
            }

            calibration.addChessboardData(chessboard.getCorners());

            cv::Mat sketch;
            chessboard.getSketch().copyTo(sketch);

            if (viewResults) { cv::imshow("Image", sketch); cv::waitKey(50); }
        }
        else if (verbose)
        {
            std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
        }
        chessboardFound.at(i) = chessboard.cornersFound();
    }
    if (viewResults) cv::destroyWindow("Image");

    if (calibration.sampleCount() < 10)
    {
        std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    double startTime = camodocal::timeInSeconds();

    calibration.calibrate();
    calibration.writeParams(cameraName + "_camera_calib.yaml");
    calibration.writeChessboardData(cameraName + "_chessboard_data.dat");

    if (verbose)
    {
        std::cout << "# INFO: Calibration took a total time of "
                  << std::fixed << std::setprecision(3) << camodocal::timeInSeconds() - startTime
                  << " sec.\n";
    }

    if (verbose)
    {
        std::cerr << "# INFO: Wrote calibration file to " << cameraName + "_camera_calib.yaml" << std::endl;
    }

    if (viewResults)
    {
        std::vector<cv::Mat> cbImages;
        std::vector<std::string> cbImageFilenames;

        for (size_t i = 0; i < imageFilenames.size(); ++i)
        {
            if (!chessboardFound.at(i))
            {
                continue;
            }

            cbImages.push_back(cv::imread(imageFilenames.at(i), -1));
            cbImageFilenames.push_back(imageFilenames.at(i));
        }

        // visualize observed and reprojected points
        calibration.drawResults(cbImages);

        for (size_t i = 0; i < cbImages.size(); ++i)
        {
            cv::putText(cbImages.at(i), cbImageFilenames.at(i), cv::Point(10,20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, cv::LINE_AA);
            cv::imshow("Image", cbImages.at(i));
            cv::waitKey(0);
        }
    }

    return 0;
}
