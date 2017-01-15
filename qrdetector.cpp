#include "qrdetector.h"

std::vector<cv::Mat> QRDetector::findQRCodes(const cv::Mat &image) const {

    // We convert the input image into greyscale, making it easier to handle.
    cv::Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));
    cvtColor(image,gray,CV_RGB2GRAY);

    // Apply the Canny operator and store the result in a new image.
    cv::Mat edgeMap(image.size(), CV_MAKETYPE(image.depth(), 1));
    Canny(gray, edgeMap, CANNY_LOWER_THRESHOLD , CANNY_UPPER_THRESHOLD);

    // Detect contours inside the edge map and store their hierarchy.
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(edgeMap, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // For each contour, retrieve the moment and calculate the center of mass.
    std::vector<cv::Point2d> centerOfMass(contours.size());
    for( int i = 0; i < contours.size(); i++ ) {
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
        if(parents >= 5) {
            positionCandidates.push_back(i);
        }
    }

    // This vector will hold the resulting qr codes.
    std::vector<cv::Mat> result;

    //---- Debug output begin
    result.push_back(gray);
    result.push_back(edgeMap);
    //---- Debug output end

    // Do not continue, if less than three markers have been found.
    if(positionCandidates.size() < 3) {
        return result;
    }

    /// Consider storing position candidates...
    std::vector<int> rating(positionCandidates.size());
    for(int i=0; i<positionCandidates.size(); i++) {
        rating[i] = 0;
        contours[positionCandidates[i]] = simplyfyContour(contours[positionCandidates[i]]);
    }

    for(int i=0; i<positionCandidates.size(); i++) {
        const std::vector<cv::Point>& contour0 = contours[i];
        for(int j=i+1; j<positionCandidates.size(); j++) {
            const std::vector<cv::Point>& contour1 = contours[j];
            for(int k=0; k<contour0.size(); k++) {
                cv::Point p0 = contour0[k];
                cv::Point p1 = contour0[(k+1) % contour0.size()];
                float mP = slope(p0, p1);

                for(int l=0; l<contour1.size(); l++) {
                    cv::Point q0 = contour1[l];
                    cv::Point q1 = contour1[(l+1) % contour1.size()];
                    float mQ = slope(q0, q1);

                    // In case the slopes do match, we count that as indicator.
                    if(std::abs(mP - mQ) < FLT_EPSILON || mP == mQ) {
                        rating[i]++;
                        rating[j]++;
                    }
                }
            }
        }
    }

    // Sort rating in order to determine the important markers.
    std::sort(rating.begin(), rating.end());

    // Just for now, draw the contours being found and filtered before.
    cv::Mat drawing = cv::Mat::zeros(edgeMap.size(), CV_MAKETYPE(edgeMap.depth(), 3));
    for(int i = 0; i< positionCandidates.size(); i++) {
        int index = positionCandidates[i];
        cv::Scalar color = cv::Scalar(255, 255, 255);
        drawContours( drawing, contours, index, color, 1, 8, hierarchy, 0, cv::Point() );
        //circle( drawing, centerOfMass[index], 4, color, -1, 8, 0 );
    }

    // Currently, only 1 qr code is supported at a time.
    // This approach simply takes the first three markers, it can find.
    std::vector<QRCode> codes;

    QRCode tmp;
    tmp.a = centerOfMass[positionCandidates[0]];
    tmp.b = centerOfMass[positionCandidates[1]];
    tmp.c = centerOfMass[positionCandidates[2]];

    codes.push_back(tmp);

    /*
    for(const QRCode& code : codes) {
        result.push_back(normalizeQRCode(image, code));
    }
    */

    //circle(drawing, tmp.a, 5, cv::Scalar(255, 0, 0), -1, 8, 0);
    //circle(drawing, tmp.b, 5, cv::Scalar(0, 255, 0), -1, 8, 0);
    //circle(drawing, tmp.c, 5, cv::Scalar(0, 0, 255), -1, 8, 0);

    for(int i=0; i<rating.size(); i++) {
        if(rating[i] > MATCHING_THRESHOLD) {
            circle(drawing, centerOfMass[positionCandidates[i]], 5, cv::Scalar(255, 0, 0), -1, 8, 0);
        }
    }

    result.push_back(drawing);

    return result;
}

cv::Mat QRDetector::normalizeQRCode(const cv::Mat& image, const QRCode& code) const {

    cv::Mat result = cv::Mat::zeros(300, 300, image.type());

    cv::Point2f srcTri[] = {code.a, code.b, code.c};
    cv::Point2f dstTri[] = {{0, 300}, {0, 0}, {300, 0}};

    cv::Mat warpMat(2, 3, CV_32FC1);
    warpMat = cv::getAffineTransform(srcTri, dstTri);

    return result;
}

std::vector<cv::Point> QRDetector::simplyfyContour(const std::vector<cv::Point>& contour) const {
    std::vector<cv::Point> simple;

    float m0 = slope(contour[0], contour[1]);
    for(int i=1; i<contour.size(); i++) {
        float m1 = slope(contour[i], contour[(i+1) % contour.size()]);

        if(std::abs(m1 - m0) > SLOPE_THRESHOLD) {
            simple.push_back(contour[i]);
        }

        m0 = m1;
    }

    return simple;
}

float QRDetector::slope(const cv::Point& p, const cv::Point& q) const {
    return (q.x != p.x) ? (q.y - p.y) / (q.x - p.x) : std::numeric_limits<float>::infinity();
}