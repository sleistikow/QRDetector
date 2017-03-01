#include "qrdetector.h"

/**
 * Helper function, calculating the squared distance between two points.
 * @param p0 the first point
 * @param p1 the second point
 * @return the squared distance
 */
static int distanceSQ(const cv::Point& p0, const cv::Point& p1) {
    cv::Point d = p1 - p0;
    return d.dot(d);
}

/**
 * Helper function, determining the orientation of the point q in relation
 * to the line defined by p0 and p1.
 * @param p0 the start of the line
 * @param p1 the end of the line
 * @param q the point to be tested
 * @return a value less than 0 if q on the right side, greater than 0 otherwise
 */
static int orientation(const cv::Point& p0, const cv::Point& p1, const cv::Point& q) {
    cv::Point n(p1.y - p0.y, p0.x - p1.x);
    return (q - p0).dot(n);
}

cv::Mat QRDetector::detectQRCode(const cv::Mat& image, int debug) const {

    // We convert the input image into greyscale, making it easier to handle.
    cv::Mat gray(image.size(), CV_8UC1);
    cv::cvtColor(image, gray, CV_RGB2GRAY);

    // Binarize the grayscale image.
    cv::adaptiveThreshold(gray, gray, 255.0, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 101, 1.0); // HACK: ugly params
    cv::medianBlur(gray, gray, MEDIAN_BLUR_NEIGHBOURHOOD); // May be helpful in some cases

    // Apply the Canny operator and store the result in a new image.
    cv::Mat edgeMap;
    cv::Canny(gray, edgeMap, CANNY_LOWER_THRESHOLD , CANNY_UPPER_THRESHOLD);

    // Detect contours inside the edge map and store their hierarchy.
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edgeMap, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Loop through hierarchy and find each contour containing two other nested contours.
    std::vector<int> positionCandidates;
    for(int i=0; i<contours.size(); i++) {
        int children = 0;
        int level = hierarchy[i][2];
        if(level >= 0) {
            double targetAreaRatio = cv::minAreaRect(contours[i]).size.area() / cv::boundingRect(contours[i]).area();
            do {
                // Discard degenerated contours.
                if(contours[level].size() < 4)
                    break;

                // Each child element needs to have the same area ratio.
                double ratio = cv::minAreaRect(contours[level]).size.area() / cv::boundingRect(contours[level]).area();
                if(std::abs(targetAreaRatio - ratio) > MAX_AREA_THRESHOLD)
                    break;

                // Found another contour on child-level!
                children++;

                // Retrieve next inlying contour.
                level = hierarchy[level][2];

            } while(level >= 0);

            // We accept each marker with more than 2 'onion'-layers around it,
            // each counting as one inner and one outer contour.
            if(children == 5)
                positionCandidates.push_back(i);
        }
    }

    // Do not continue, if less than three markers have been found.
    if(positionCandidates.size() < 3) {
        return cv::Mat();
    }

    // Create inpainting image.
    cv::Mat augmented = image.clone();

    // Simplify the retrieved contours and draw debug visuals.
    std::vector<std::vector<cv::Point>> contourCandidates;
    for(int i : positionCandidates) {

        // Simplify contour.
        contours[i] = simplifyContour(contours[i], 4);

        // Draw the contours being found.
        cv::drawContours(augmented, contours, i, cv::Scalar(255, 255, 255), 2, 8, hierarchy, 0);

        // Store candidate
        contourCandidates.push_back(contours[i]);
    }

    // Don't continue, if more than 3 contour candidates have been found.
    if(contourCandidates.size() > 3) {
        std::cout << "More than 3 markers have been found! Only one QR Code is supported at a time." << std::endl;
        return cv::Mat();
    }

    // Currently, only 1 qr code is supported at a time.
    // This approach simply takes the first three markers, it can find.
    QRCode code = extractQRCode(contourCandidates, augmented);

    // Align code.
    cv::Mat imAligned = alignQRCode(image, code);

    // Normalize code.
    cv::Mat imNormalized = normalizeQRCode(imAligned, code);

    //---- Debug output begin
    if(debug & DEBUG_BINARY)    cv::imshow("gray", gray);
    //if(debug & DEBUG_EDGE)      cv::imshow("edge", edgeMap);
    if(debug & DEBUG_ALIGNED)   cv::imshow("aligned", imAligned);
    if(debug & DEBUG_AUGMENTED) cv::imshow("input", augmented);
    //---- Debug output end

    // Return the normalized result.
    return imNormalized;
}

QRDetector::QRCode QRDetector::extractQRCode(const std::vector<std::vector<cv::Point>>& patterns, cv::Mat& augmented) const {
    assert(patterns.size() == 3); // At this point, we must have three markers sorted out.

    // For each contour, calculate the center of mass.
    std::vector<cv::Point2d> centerOfMass(patterns.size());
    std::vector<cv::Point> corners;
    for(int i = 0; i<patterns.size(); i++) {
        cv::Moments m = cv::moments(patterns[i], false);
        centerOfMass[i] = cv::Point2d(m.m10/m.m00 , m.m01/m.m00);
        cv::circle(augmented, centerOfMass[i], 4, cv::Scalar(255, 255, 255), -1, 8, 0);

        // Add all vertices to the global square contour.
        for(int j=0; j<patterns[i].size(); j++)
            corners.push_back(patterns[i][j]);
    }

    // Find the corner just after the lower left marker, being determined by the longest distance.
    int maxDist = distanceSQ(centerOfMass[0], centerOfMass[2]), dist;
    int idxC = (orientation(centerOfMass[0], centerOfMass[2], centerOfMass[1]) < 0) ? 0 : 2;
    if((dist = distanceSQ(centerOfMass[0], centerOfMass[1])) > maxDist) {
        maxDist = dist;
        idxC = (orientation(centerOfMass[0], centerOfMass[1], centerOfMass[2]) < 0) ? 0 : 1;
    }
    if(distanceSQ(centerOfMass[1], centerOfMass[2]) > maxDist) {
        idxC = (orientation(centerOfMass[1], centerOfMass[2], centerOfMass[0]) < 0) ? 1 : 2;
    }

    cv::circle(augmented, centerOfMass[idxC], 10, cv::Scalar(0, 0, 255), -1, 8, 0);

    // Simplify square to determine enclosing rect.
    std::vector<std::vector<cv::Point>> hullContour(1);
    std::vector<cv::Point>& hull = hullContour[0];
    hull = simplifyContour(corners, 5);
    cv::drawContours(augmented, hullContour, 0, cv::Scalar(255, 0, 0), 2, 8, -1, 0);

    // Since the start point has been lost in the simplification process,
    // we need to determine the indices of the necessary points again.
    int start = -1;
    for(int i=0; (start == -1) && i<hull.size(); i++) {
        for(int j=0; j<patterns[idxC].size(); j++) {
            if(distanceSQ(hull[i], patterns[idxC][j]) <= 4 && // Position matches
               orientation(hull[i], hull[(i+1) % hull.size()], patterns[idxC][(j+1)%patterns[idxC].size()]) < 0) // Location inside pattern matches
            {
                start = i;
                break; // Index is clear, we can break here.
            }
        }
    }

    // We always must find the correct position!
    assert(start >= 0);

    // Get helper points.
    cv::Point helpC = hull[(start + 0) % hull.size()];
    cv::Point helpA = hull[(start + 1) % hull.size()];

    // Build the QR code.
    QRCode code;
    code.a = hull[(start + 2) % hull.size()];
    code.b = hull[(start + 3) % hull.size()];
    code.c = hull[(start + 4) % hull.size()];
    code.d = intersect(code.a, helpA, code.c, helpC);

    // Draw the selected corners for debug assessment.
    cv::circle(augmented, code.a, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
    cv::circle(augmented, code.b, 4, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::circle(augmented, code.c, 4, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(augmented, code.d, 4, cv::Scalar(255, 255, 255), -1, 8, 0);

    code.patterns = patterns;

    // Done here.
    return code;
}

cv::Mat QRDetector::alignQRCode(const cv::Mat& image, QRCode& code) const {

    cv::Mat result(QR_BUFFER_SIZE, QR_BUFFER_SIZE, image.type());
    const float size = QR_BUFFER_SIZE;

    cv::Point2f srcQuad[] = {code.a, code.b, code.c, code.d};
    cv::Point2f dstQuad[] = {
            {0.0f,  size},
            {0.0f,  0.0f},
            {size,  0.0f},
            {size,  size},
    };

    // Transform image.
    cv::Mat warpMat = cv::getPerspectiveTransform(srcQuad, dstQuad);
    cv::warpPerspective(image, result, warpMat, result.size());

    // Determine cell size in image space.
    // 1. Convert vertices to floating point.
    std::vector<cv::Point2f> transformed(3*4); // patterns * corners
    for(int i=0; i<code.patterns.size(); i++) {
        for(int j=0; j<code.patterns[i].size(); j++) {
            transformed.push_back(code.patterns[i][j]);
        }
    }

    // 2. Transform vertices.
    cv::perspectiveTransform(transformed, transformed, warpMat);

    // 3. Determine average distance between corners.
    float len = 0.0f;
    for(int i=0; i<transformed.size(); i+=4) {
        for(int j=0; j<4; j++) {
            len += std::sqrt(distanceSQ(transformed[i+j], transformed[i+(j+1) % 4]));
        }
    }
    code.cellsize = len / (7 * 3 * 4); // cells * markers * distances/marker

    return result;
}

cv::Mat QRDetector::normalizeQRCode(const cv::Mat& image, const QRCode& code) const {

    // First, convert into binary image.
    cv::Mat binary;
    //FIXME: binarize beforehand?
    //cv::threshold(image, binary, BINARY_THRESHOLD, 255, cv::THRESH_OTSU);
    cv::cvtColor(image, binary, CV_RGB2GRAY);
    cv::adaptiveThreshold(binary, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 101, 1.0); // HACK: ugly code

    // Determine code size.
    int size = static_cast<int>(QR_BUFFER_SIZE / code.cellsize);
    float d = static_cast<float>(QR_BUFFER_SIZE) / size;

    cv::Mat final = cv::Mat::zeros(size, size, CV_8UC1);
    for(int r=0; r<size; r++) {
        for(int c=0; c<size; c++) {
            // In case cells are big enough, take neighbourhood into account.
            if(code.cellsize > 3.0f) {
                int avg = 0;
                for(int i=-1; i<=1; i++) {
                    for(int j=-1; j<=1; j++) {
                        int x = static_cast<int>(d * (r + 0.5f)) + i;
                        int y = static_cast<int>(d * (c + 0.5f)) + j;
                        avg += binary.at<uchar>(x, y);
                    }
                }

                if(avg / 9 > BINARY_THRESHOLD)
                    final.at<uchar>(r, c) = 255;

            // Otherwise, pick a single pixel.
            } else {
                int x = static_cast<int>(d * (r + 0.5f));
                int y = static_cast<int>(d * (c + 0.5f));
                if(binary.at<uchar>(x, y) > BINARY_THRESHOLD)
                    final.at<uchar>(r, c) = 255;
            }
        }
    }

    return final;
}

std::vector<cv::Point> QRDetector::simplifyContour(const std::vector<cv::Point>& contour, int numPoints) const {
    std::vector<cv::Point> simple;

    // Convex hull for a first approx result.
    cv::convexHull(contour, simple, false);

    // Simplify further, until the desired amount of corners has been extracted.
    double epsilon = SIMPLFIICATION_START_EPSILON;
    while(simple.size() > numPoints) {
        cv::approxPolyDP(simple, simple, epsilon, true);
        epsilon *= 2.0;
    }
    return simple;
}

cv::Point QRDetector::intersect(const cv::Point& a0, const cv::Point& a1, const cv::Point& c0, const cv::Point& c1) const {

    //TODO: handle a1.x == a0.x || c1.x == c0.x

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
