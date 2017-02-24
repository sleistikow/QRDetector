#include "qrdetector.h"

cv::Mat QRDetector::findQRCode(const cv::Mat &image) {

    // We convert the input image into greyscale, making it easier to handle.
    cv::Mat gray(image.size(), CV_8UC1);
    cv::cvtColor(image, gray, CV_RGB2GRAY);

    // Apply the Canny operator and store the result in a new image.
    cv::Mat edgeMap;//(image.size(), CV_8UC1);
    cv::Canny(gray, edgeMap, CANNY_LOWER_THRESHOLD , CANNY_UPPER_THRESHOLD);

    // Detect contours inside the edge map and store their hierarchy.
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edgeMap, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // For each contour, retrieve the moment and calculate the center of mass.
    std::vector<cv::Point2d> centerOfMass(contours.size());
    for(int i = 0; i<contours.size(); i++) {
        // retrieve the moments
        cv::Moments m = moments(contours[i], false);
        // calculate the mass center of the moments
        centerOfMass[i] = cv::Point2d(m.m10/m.m00 , m.m01/m.m00);
    }

    // Loop through hierarchy and find each contour containing two other nested contours.
    std::vector<int> positionCandidates;
    for(int i=0; i<contours.size(); i++) {
        int parents = 0;
        int level = hierarchy[i][2];
        while(level >= 0) {
            level = hierarchy[level][2];
            parents++;
        }

        // We accept each marker with more than 2 'onion'-layers around it.
        if(parents >= 5 && contours[i].size() > 0) {
            positionCandidates.push_back(i);
        }
    }

    // Do not continue, if less than three markers have been found.
    if(positionCandidates.size() != 3) {
        if(positionCandidates.size() > 3)
            std::cout << "More than 3 markers have been found! Only one QR Code is supported at a time." << std::endl;
        return cv::Mat();
    }

    // Create inpainting image.
    inpainting = image.clone();

    // Simplify the retrieved contours and prepare enclosing square calculation.
    std::vector<cv::Point> corners;
    for(int i : positionCandidates) {

        // Simplify contour.
        contours[i] = simplyfyContour(contours[i]);

        // Draw the contours being found.
        cv::Scalar color = cv::Scalar(255, 255, 255);
        cv::drawContours(inpainting, contours, i, color, 2, 8, hierarchy, 0);
        cv::circle(inpainting, centerOfMass[i], 4, color, -1, 8, 0);

        // Add all vertices to the global square contour.
        for(int j=0; j<contours[i].size(); j++)
            corners.push_back(contours[i][j]);
    }

    // Currently, only 1 qr code is supported at a time.
    // This approach simply takes the first three markers, it can find.
    QRCode code = extractQRCode(corners);

    // Align code.
    cv::Mat imAligned = alignQRCode(gray, code);

    //---- Debug output begin
    //cv::imshow("gray", gray);
    //cv::imshow("edge", edgeMap);
    //cv::imshow("aligned", imAligned);
    cv::imshow("input", inpainting);
    //---- Debug output end

    // Return the normalized result.
    return normalizeQRCode(imAligned, code);
}

static int distanceSQ(const cv::Point& p0, const cv::Point& p1) {
    cv::Point d = p1 - p0;
    return d.dot(d);
}

QRDetector::QRCode QRDetector::extractQRCode(const std::vector<cv::Point>& corners) {

    // Simplify square to determine enclosing rect.
    std::vector<std::vector<cv::Point>> squareContour(1);
    std::vector<cv::Point>& square = squareContour[0];
    cv::convexHull(corners, square, false); // Note: counter-clockwise
    cv::approxPolyDP(square, square, 5.0, true);
    //cv::convexHull(square, square, false); // Note: counter-clockwise
    cv::drawContours(inpainting, squareContour, 0, cv::Scalar(255, 0, 0), 2, 8, -1, 0);

    // Find the corner just after the lower left marker, being determined by the longest distance.
    int start = 0;
    int maxDist = 0;
    for(int i=0; i<square.size(); i++) {
        // TODO: does not always work properly!
        int dist = distanceSQ(square[i], square[(i+1)%square.size()]);
        if(dist > maxDist) {
            maxDist = dist;
            start = i;
        }
    }

    // Get helper points.
    cv::Point helpA = square[(start + 1) % square.size()];
    cv::Point helpC = square[(start + 0) % square.size()];

    // Build the QR code.
    QRCode code;
    code.a = square[(start + 2) % square.size()];
    code.b = square[(start + 3) % square.size()];
    code.c = square[(start + 4) % square.size()];
    code.d = intersect(code.a, helpA, code.c, helpC);

    // Draw the selected corners for debug assessment.
    cv::circle(inpainting, code.a, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
    cv::circle(inpainting, code.b, 4, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::circle(inpainting, code.c, 4, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(inpainting, code.d, 4, cv::Scalar(255, 255, 255), -1, 8, 0);

    // Determine cell size in image space.
    //TODO: pretend the outer length is 7 - true??
    code.cellsize = (sqrtf(distanceSQ(code.a, helpA)) + sqrtf(distanceSQ(code.c, helpC))) / (2.0f * 7.0f);

    // Done here.
    return code;
}

cv::Mat QRDetector::alignQRCode(const cv::Mat& image, const QRCode& code) const {

    cv::Mat result(QR_BUFFER_SIZE, QR_BUFFER_SIZE, image.type());
    const float size = QR_BUFFER_SIZE;

    cv::Point2f srcQuad[] = {code.a, code.b, code.c, code.d};
    cv::Point2f dstQuad[] = {
            {0.0f,  size},
            {0.0f,  0.0f},
            {size,  0.0f},
            {size,  size},
    };

    cv::Mat warpMat(2, 3, CV_32FC1);
    warpMat = cv::getAffineTransform(srcQuad, dstQuad);
    cv::warpAffine(image, result, warpMat, result.size());

    return result;
}

cv::Mat QRDetector::normalizeQRCode(const cv::Mat& image, const QRCode& code) const {

    // First, convert into binary image.
    cv::Mat binary;
    //cv::threshold(image, result, BINARY_THRESHOLD, 255, cv::THRESH_BINARY);
    cv::adaptiveThreshold(image, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 101, 1.0); // HACK: ugly code

    /*
    int size = static_cast<int>((image.rows + image.cols) / (2.0f * code.cellsize));
    cv::resize(binary, binary, cv::Size(size, size), 0.0, 0.0, CV_INTER_AREA);
    return binary;
    /*/

    // Determine code size.
    int size = static_cast<int>(0.5f * QR_BUFFER_SIZE / code.cellsize); // FIXME: Why is it we need to mult by 0.5 ??
    float d = static_cast<float>(QR_BUFFER_SIZE) / size;

    cv::Mat final = cv::Mat::zeros(size, size, CV_8UC1);
    for(int r=0; r<size; r++) {
        for(int c=0; c<size; c++) {
            int x = static_cast<int>(d * (r + 0.5f));
            int y = static_cast<int>(d * (c + 0.5f));
            if(binary.at<uchar>(x, y) > 0)
                final.at<uchar>(r, c) = 255;
        }
    }

    return final;
    //*/
}

std::vector<cv::Point> QRDetector::simplyfyContour(const std::vector<cv::Point>& contour) const {
    std::vector<cv::Point> simple;

    // Convex hull for a first approx result.
    cv::convexHull(contour, simple, false);

    // Simplify further, until 4 corners have been extracted.
    double epsilon = SIMPLFIICATION_START_EPSILON;
    while(simple.size() > 4) {
        cv::approxPolyDP(simple, simple, epsilon, true);
        epsilon *= 2.0;
    }
    return simple;
}

cv::Point QRDetector::intersect(const cv::Point& a0, const cv::Point& a1, const cv::Point& c0, const cv::Point& c1) const {

    // Calculate slopes.
    double mA = (a1.y - a0.y) / static_cast<double>(a1.x - a0.x);
    double mC = (c1.y - c0.y) / static_cast<double>(c1.x - c0.x);

    // Calculate axis intersection.
    double bA = a0.y - (mA * a0.x);
    double bC = c0.y - (mC * c0.x);

    // Calculate line intersection point.
    double x = (bC - bA) / (mA - mC);
    double y = mC * x + bC;

    return cv::Point(static_cast<int>(x), static_cast<int>(y));
}
