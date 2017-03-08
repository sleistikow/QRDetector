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

    // Binarize and filter the grayscale image.
    cv::medianBlur(gray, gray, MEDIAN_BLUR_NEIGHBOURHOOD);
    cv::threshold(gray, gray, BINARY_THRESHOLD, 255, cv::THRESH_OTSU);
    cv::medianBlur(gray, gray, MEDIAN_BLUR_NEIGHBOURHOOD);

    // Apply the Canny operator and store the result in a new image.
    cv::Mat edgeMap;
    cv::Canny(gray, edgeMap, CANNY_LOWER_THRESHOLD , CANNY_UPPER_THRESHOLD);

    // Detect contours inside the edge map and store their hierarchy.
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edgeMap, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Loop through hierarchy and find each contour containing two other nested contours.
    double ratios[] = {1.0, 7.0/5.0, 1.0, 5.0/3.0, 1.0};
    std::vector<int> positionCandidates;
    for(int i=0; i<contours.size(); i++) {
        int children = 0;
        int level = hierarchy[i][2];
        if(level >= 0) {
            double outerArea = cv::minAreaRect(contours[i]).size.area();
            do {
                // Discard degenerated contours.
                if(contours[level].size() < 4)
                    break;

                // Each child element needs to have the same area ratio.
                double innerArea = cv::minAreaRect(contours[level]).size.area();
                if(std::abs((outerArea / innerArea) - ratios[children]) > MAX_AREA_THRESHOLD)
                    break;
                outerArea = innerArea;

                // Found another valid contour on child-level!
                children++;

                // Retrieve next inlying contour.
                level = hierarchy[level][2];

            } while(level >= 0);

            // We accept each contour with exactly 5 inlying contours,
            // which nearly only applies to markers.
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
        cv::drawContours(augmented, contours, i, cv::Scalar(0, 0, 255), 2, 8);

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
    if(debug & DEBUG_EDGE)      cv::imshow("edge", edgeMap);
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

    // Simplify contour to determine enclosing hull.
    std::vector<std::vector<cv::Point>> hullContour(1);
    std::vector<cv::Point>& hull = hullContour[0];
    float epsilon;
    hull = simplifyContour(corners, 5, &epsilon);
    cv::drawContours(augmented, hullContour, 0, cv::Scalar(255, 0, 0), 2, 8, -1, 0);

    // Since the start point has been lost in the simplification process,
    // we need to determine the indices of the necessary points again.
    int start = -1;
    for(int i=0; (start == -1) && i<hull.size(); i++) {
        for(int j=0; j<patterns[idxC].size(); j++) {
            if(distanceSQ(hull[i], patterns[idxC][j]) <= epsilon*epsilon && // Position matches
               orientation(hull[i], hull[(i+1) % hull.size()], patterns[idxC][(j+1)%patterns[idxC].size()]) < 0) // Location inside pattern matches
            {
                start = i;
                break; // Index is clear, we can break here.
            }
        }
    }

    // We always must find the correct position!
    //assert(start >= 0);
    if(start == -1) start = 0; // HACK: even if unlikely, better continue working.

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

    // Store patterns for later use.
    code.patterns = patterns;

    // Done here.
    return code;
}

cv::Mat QRDetector::alignQRCode(const cv::Mat& image, QRCode& code) const {

    // Choose size such that no data gets lost during transformation.
    const float size = std::ceil(std::sqrt(std::max({distanceSQ(code.a, code.b),
                                                     distanceSQ(code.b, code.c),
                                                     distanceSQ(code.c, code.d),
                                                     distanceSQ(code.d, code.a)}))
                                 );
    cv::Mat result(static_cast<int>(size), static_cast<int>(size), image.type());

    // Define source and target vertices.
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
    float s = 0;
    for(int i=0; i<code.patterns.size(); i++) {
        // 1. Convert vertices to floating point.
        std::vector<cv::Point2f> transformed(4);
        for(int j=0; j<code.patterns[i].size(); j++) {
            transformed[j] = code.patterns[i][j];
        }

        // 2. Transform vertices.
        cv::perspectiveTransform(transformed, transformed, warpMat);

        // 3. Retrieve size of the minimal bounding rect.
        cv::Size2f size = cv::minAreaRect(transformed).size;
        s += size.width + size.height;
    }
    code.moduleSize = s / (7 * 3 * 2); // modules/marker * marker * box sides

    return result;
}

cv::Mat QRDetector::normalizeQRCode(const cv::Mat& image, const QRCode& code) const {

    // First, convert into binary image.
    cv::Mat binary;
    cv::cvtColor(image, binary, CV_RGB2GRAY);
    cv::threshold(binary, binary, BINARY_THRESHOLD, 255, cv::THRESH_OTSU);

    // Determine code size. Since the code has 17 + n * 4 modules per dimension, the approximation gets more accurat.
    int size = static_cast<int>(std::round((image.rows / code.moduleSize - 17) / 4.0f)) * 4 + 17;
    //int size = countColorSwitch(binary, cv::Point(code.moduleSize*0.5f, code.moduleSize*6.5f), cv::Point(image.rows-code.moduleSize*0.5f, code.moduleSize*6.5f)) - 1 + 14;
    float moduleSize = static_cast<float>(image.rows) / size;

    cv::Mat final = cv::Mat::zeros(size, size, CV_8UC1);
    for(int r=0; r<size; r++) {
        for(int c=0; c<size; c++) {

            // Sample point in the middle of a module.
            int x = static_cast<int>(moduleSize * (r + 0.5f));
            int y = static_cast<int>(moduleSize * (c + 0.5f));

            // In case cells are big enough, take neighbourhood into account.
            if(moduleSize > 3.0f) {
                final.at<uchar>(r, c) = pickMeanPixel(binary, cv::Point(x, y), 1);
            // Otherwise, pick a single pixel.
            } else {
                if(binary.at<uchar>(x, y) > BINARY_THRESHOLD)
                    final.at<uchar>(r, c) = 255;
            }
        }
    }

    return final;
}

std::vector<cv::Point> QRDetector::simplifyContour(const std::vector<cv::Point>& contour, int numPoints, float* const eps) const {
    std::vector<cv::Point> simple = contour;

    // Convex hull for a first approx result.
    cv::convexHull(contour, simple, false);

    // Simplify further, until the desired amount of corners has been extracted.
    double epsilon = SIMPLFIICATION_START_EPSILON;
    while(simple.size() > numPoints) {
        cv::approxPolyDP(simple, simple, epsilon, true);
        epsilon += 1.0;
    }

    // Store the epsilon that made it.
    if(eps)
        *eps = epsilon;

    return simple;
}

int QRDetector::countColorSwitch(const cv::Mat& image, const cv::Point& start, const cv::Point& end) const {
    assert(image.type() == CV_8UC1);

    int count = 0;

    // Adapted from Midpoint-line algorithm.
    int dx = std::abs(end.x - start.x);
    int sx = (start.x < end.x ? 1 : -1);
    int dy = abs(end.y - start.y);
    int sy = (start.y < end.y ? 1 : -1);
    int dE = 2 * dy;
    int d = 2 * dy - dx;
    int dNE = 2 * (dy - dx);
    int x = start.x, y = start.y;
    uchar color = image.at<uchar>(y, x);

    while (x != end.x) {
        if ((d < 0) || ((d == 0) && (sx == 1))) {
            d += dE; x += sx;
        } else {
            d += dNE; x += sx; y += sy;
        }
        uchar c = pickMeanPixel(image, cv::Point(y, x), 3);
        if(c != color) {
            color = c;
            count++;
        }
    }

    return count;
}

uchar QRDetector::pickMeanPixel(const cv::Mat& image, const cv::Point& position, int k) const {
    assert(image.type() == CV_8UC1);

    uint avg = 0;
    for(int i=-k; i<=k; i++) {
        for(int j=-k; j<=k; j++) {
            avg += image.at<uchar>(position.x+i, position.y+j);
        }
    }

    if(avg / (((k+2)*(k+2))) > BINARY_THRESHOLD)
        return 255;

    return 0;
}

cv::Point QRDetector::intersect(const cv::Point& a0, const cv::Point& a1, const cv::Point& c0, const cv::Point& c1) const {

    if(c1.x == c0.x) {
        if(a1.y != a0.y) {
            double mA = (a1.y - a0.y) / static_cast<double>(a1.x - a0.x);
            double bA = a0.y - (mA * a0.x);
            double y = mA * c1.x + bA;
            return cv::Point(c1.x, static_cast<int>(y));
        } else {
            return cv::Point(c1.x, a1.y);
        }
    } else if(a1.x == a0.x) {
        if(c1.y != c0.y) {
            double mC = (c1.y - c0.y) / static_cast<double>(c1.x - c0.x);
            double bC = c0.y - (mC * c0.x);
            double y = mC * a0.x + bC;
            return cv::Point(a1.x, y);
        } else {
            return cv::Point(a1.x, c1.y);
        }
    } else {
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
}
