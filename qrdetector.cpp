#include "qrdetector.h"

cv::Mat QRDetector::findQRCode(const cv::Mat &image) const {

    // We convert the input image into greyscale, making it easier to handle.
    cv::Mat gray(image.size(), CV_8UC1);
    cvtColor(image, gray, CV_RGB2GRAY);

    // Apply the Canny operator and store the result in a new image.
    cv::Mat edgeMap(image.size(), CV_8UC1);
    Canny(gray, edgeMap, CANNY_LOWER_THRESHOLD , CANNY_UPPER_THRESHOLD);

    // Detect contours inside the edge map and store their hierarchy.
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edgeMap, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

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
        if(parents >= 5 && contours[i].size() > 0) {
            positionCandidates.push_back(i);
        }
    }

    // Do not continue, if less than three markers have been found.
    if(positionCandidates.size() < 3) {
        return cv::Mat();
    }

    /// Consider storing position candidates...
    std::vector<int> rating(positionCandidates.size());
    for(int i=0; i<positionCandidates.size(); i++) {
        rating[i] = 0;
        contours[positionCandidates[i]] = simplyfyContour(contours[positionCandidates[i]]);
        //std::cout << contours[positionCandidates[i]].size() << std::endl;
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
    cv::Mat drawing = image.clone();//cv::Mat::zeros(edgeMap.size(), CV_MAKETYPE(edgeMap.depth(), 3));
    for(int i = 0; i< positionCandidates.size(); i++) {
        int index = positionCandidates[i];
        cv::Scalar color = cv::Scalar(255, 255, 255);
        cv::drawContours( drawing, contours, index, color, 1, 8, hierarchy, 0, cv::Point() );
        circle( drawing, centerOfMass[index], 4, color, -1, 8, 0 );
    }

    /*
    for(int i=0; i<rating.size(); i++) {
        if(rating[i] > MATCHING_THRESHOLD) {
            circle(drawing, centerOfMass[positionCandidates[i]], 5, cv::Scalar(255, 0, 0), -1, 8, 0);
        }
    }
    */

    // Currently, only 1 qr code is supported at a time.
    // This approach simply takes the first three markers, it can find.
    QRCode tmp;
    tmp.a = centerOfMass[positionCandidates[0]];
    tmp.b = centerOfMass[positionCandidates[1]];
    tmp.c = centerOfMass[positionCandidates[2]];
    tmp.d = intersect(tmp.b, tmp.c, tmp.a);

    circle(drawing, tmp.a, 5, cv::Scalar(255, 0, 0), -1, 8, 0);
    circle(drawing, tmp.b, 5, cv::Scalar(0, 255, 0), -1, 8, 0);
    circle(drawing, tmp.c, 5, cv::Scalar(0, 0, 255), -1, 8, 0);
    circle(drawing, tmp.d, 5, cv::Scalar(255, 0, 255), -1, 8, 0);

    //---- Debug output begin
    //cv::imshow("gray", gray);
    //cv::imshow("edge", edgeMap);
    cv::imshow("input", drawing);
    //---- Debug output end

    // Return the normalized result.
    return normalizeQRCode(image, tmp);
}

cv::Mat QRDetector::normalizeQRCode(const cv::Mat& image, const QRCode& code) const {

    cv::Mat result = cv::Mat::zeros(400, 400, image.type());

    cv::Point2f srcTri[] = {code.c, code.b, code.a, code.d};
    float o = 0.15f * result.rows;
    cv::Point2f dstTri[] = {
            {0.0f + o, 0.0f + o},
            {(result.rows-1) - o, 0.0f + o},
            {0.0f + o, result.cols-1 - o},
            {(result.rows-1) - o, (result.cols-1) - o}
    };

    cv::Mat warpMat(2, 3, CV_32FC1);
    warpMat = cv::getAffineTransform(srcTri, dstTri);
    cv::warpAffine(image, result, warpMat, result.size());

    return result;
}

static int distanceSQ(const cv::Point& p0, const cv::Point& p1) {
    cv::Point d = p1 - p0;
    return d.dot(d);
}

std::vector<cv::Point> QRDetector::simplyfyContour(const std::vector<cv::Point>& contour) const {
    std::vector<cv::Point> simple;

    cv::convexHull(contour, simple, false);
    if(simple.size() > 4) {
        //std::cout << "Too large: " << simple.size() << std::endl;
        //std::vector<cv::Point> simpler;

        // Ansatz 1:
        // Prüfe den Abstand zweier aufeinanderfolgender Punkte.
        // Ist dieser deutlich (!) kleiner als der zu anderen Punkten, lösche ihn.

        // Ansatz 2:
        // Siehe einen Punkt aus contour als Startpunkt an
        // Suche den Punkt mit dem größten Abstand, wenn dieser Punkt zuvor noch nicht gefunden wurde.
/*
        std::map<int, std::pair<int, int> > result;
        for(int k=0; k<contour.size(); k++) {
            int maxDist = 0;
            int idx = 0;
            for(int i=0; i<contour.size(); i++) {
                int d = distanceSQ(contour[0], contour[i]);
                if(d > maxDist) {
                    maxDist = d;
                    idx = i;
                }
            }
            result[maxDist] = std::make_pair(k, idx);
        }

        auto first = result.begin();
        auto second = ++result.begin();
        simple.push_back(contour[(*first).second.first]);
        simple.push_back(contour[(*second).second.first]);
        simple.push_back(contour[(*first).second.second]);
        simple.push_back(contour[(*second).second.second]);
*/
        //simple = simpler;
    }
    return simple;
}

float QRDetector::slope(const cv::Point& p, const cv::Point& q) const {
    return (q.x != p.x) ? (q.y - p.y) / static_cast<float>(q.x - p.x) : std::numeric_limits<float>::infinity();
}

cv::Point QRDetector::intersect(const cv::Point& a, const cv::Point& b, const cv::Point& c) const {

    float m1 = (b.y - a.y) / static_cast<float>(b.x - a.x);
    float m2 = (c.y - b.y) / static_cast<float>(c.x - b.x);

    float b1 = a.y - (m2 * a.x);
    float b2 = c.y - (m1 * c.x);

    float x = (b2 - b1) / (m2 - m1);
    float y = m2 * x + b1;

    return cv::Point(static_cast<int>(x), static_cast<int>(y));
}
