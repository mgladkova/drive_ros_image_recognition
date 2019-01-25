#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include <functional>
#include "drive_ros_image_recognition/line_detection.h"
#include "drive_ros_image_recognition/geometry_common.h"
#include "drive_ros_msgs/RoadLine.h"
#include "drive_ros_msgs/simple_trajectory.h"
#include "drive_ros_msgs/DrivingLine.h"

namespace drive_ros_image_recognition {

LineDetection::LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , imageTransport_(pnh)
    , laneWidthWorld_(0.4)
	, laneVar_(0.1)
    , segmentLength_(0.2)
	, maxSenseRange_(1.6)
	, maxRansacInterations_(200)
    , image_operator_()
    , dsrv_server_()
    , dsrv_cb_(boost::bind(&LineDetection::reconfigureCB, this, _1, _2))
    , roadModel(&tfListener_, laneWidthWorld_)
{
}

LineDetection::~LineDetection() {
}

///
/// \brief LineDetection::init
/// Sets dynamic reconfigure server, subscribes to camera image, inits image_operator.
/// \return true for success, false for failure.
///
bool LineDetection::init() {
    // dynamic reconfigure
    dsrv_server_.setCallback(dsrv_cb_);

    //subscribe to camera image
    imageSubscriber_ = imageTransport_.subscribe("/img_in", 3, &LineDetection::imageCallback, this);
    ROS_INFO_STREAM("Subscribed image transport to topic " << imageSubscriber_.getTopic());

    odometrySub = pnh_.subscribe("odom_topic", 3, &LineDetection::odometryCallback, this);
    ROS_INFO("Subscribing to odometry on topic '%s'", odometrySub.getTopic().c_str());

    drivingLinePub = nh_.advertise<drive_ros_msgs::DrivingLine>("driving_line_topic", 1);
    ROS_INFO("Publish driving line on topic '%s'", drivingLinePub.getTopic().c_str());

#ifdef PUBLISH_DEBUG
    debugImgPub_ = imageTransport_.advertise("debug_image", 3);
    ROS_INFO_STREAM("Publishing debug image on topic " << debugImgPub_.getTopic());
#endif

    // common image operations
    if(!image_operator_.init()) {
        ROS_WARN_STREAM("Failed to init image_operator");
        return false;
    }

    return true;
}

void LineDetection::odometryCallback(const nav_msgs::OdometryConstPtr &odomMsg) {
	latestOdometry = *odomMsg;
}

///
/// \brief LineDetection::imageCallback
/// Called for incoming camera image. Extracts the image from the incoming message.
/// \param imgIn the incoming camera image message.
///
void LineDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
    auto currentImage = convertImageMessage(imgIn);
    cv::Mat homographedImg;
    if(!image_operator_.homographImage(*currentImage, homographedImg)) {
        ROS_WARN("Homographing image failed");
        return;
    }
    imgHeight_ = homographedImg.rows;
    imgWidth_ = homographedImg.cols;
//    imgTimestamp = imgIn->header.stamp;
    imgTimestamp = ros::Time::now();

#ifdef PUBLISH_DEBUG
    cv::cvtColor(homographedImg, debugImg_, CV_GRAY2RGB);
#endif

    // Find lines in the image with Hough
    std::vector<Line> linesInImage;
    findLinesWithHough(homographedImg, linesInImage);

    if(linesInImage.empty()) {
    	ROS_ERROR("No Hough lines found");
    	return;
    }

#ifdef PUBLISH_DEBUG
    if(drawDebugLines_) {
        // Draw the Hough lines
        for(auto l : linesInImage) {
            cv::line(debugImg_, l.iP1_, l.iP2_, cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 2);
        }

        // Display the given lane width
        std::vector<cv::Point2f> worldPts, imagePts;
        worldPts.push_back(cv::Point2f(0.2f, -.5f * laneWidthWorld_));
        worldPts.push_back(cv::Point2f(0.2f, .5f * laneWidthWorld_));

        image_operator_.worldToWarpedImg(worldPts, imagePts);

        cv::line(debugImg_, imagePts.at(0), imagePts.at(1), cv::Scalar(0,0,255), 4, cv::LINE_AA);
    }
#endif

    findLaneMarkings(linesInImage);

    // Publish the driving line message
    float detectionRange = .0f;
    drive_ros_msgs::DrivingLine drivingLineMsg;
    auto drivingLinePoly = roadModel.getDrivingLinePts(detectionRange);
    drivingLineMsg.detectionRange = detectionRange;
    drivingLineMsg.polynom_order = drivingLinePoly.getOrder();

    for(auto c : drivingLinePoly.getCoeffs()) {
    	drivingLineMsg.polynom_params.push_back(c);
    }

    drivingLinePub.publish(drivingLineMsg);

#ifdef PUBLISH_DEBUG
    std::vector<cv::Point2f> worldPts, imgPts;

    for(float x = 0.f; x < detectionRange; x += .2f) {
    	worldPts.push_back(cv::Point2f(x, drivingLinePoly.atX(x)));
    }
    worldPts.push_back(cv::Point2f(detectionRange, drivingLinePoly.atX(detectionRange)));

    image_operator_.worldToWarpedImg(worldPts, imgPts);

    for(int i = 1; i < imgPts.size(); i++) {
    	cv::line(debugImg_, imgPts.at(i-1), imgPts.at(i), cv::Scalar(0,255), 2, cv::LINE_AA);
    }
    debugImgPub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, debugImg_).toImageMsg());
#endif

    //  auto t_start = std::chrono::high_resolution_clock::now();
    //  findLane();
    //  auto t_end = std::chrono::high_resolution_clock::now();
    //  ROS_INFO_STREAM("Cycle time: " << (std::chrono::duration<double, std::milli>(t_end-t_start).count()) << "ms");
}

void LineDetection::findLaneMarkings(std::vector<Line> &lines) {
    cv::Point2f segStartWorld(0.3f, 0.f);
    float segAngle = 0.f;
    float totalSegLength = 0.f;
    std::vector<Segment> foundSegments;
    std::vector<Line*> unusedLines, leftMarkings, midMarkings, rightMarkings, otherMarkings;
    std::vector<cv::RotatedRect> regions;

    ROS_INFO("------------- New image ------------------");

    for(int i = 0; i < lines.size(); i++) {
    	unusedLines.push_back(&(lines.at(i)));
    }

    // ================================
    // Find lane in new image
    // ================================
    std::vector<cv::Point2f> segmentStarts;
    std::vector<float> segmentAngles;
    int numSegmentsToUse = maxSenseRange_ / segmentLength_;
    bool useRoadModelSegment = true;
    roadModel.getSegmentPositions(segmentStarts, segmentAngles, imgTimestamp);
    roadModel.getSegmentSearchStart(segStartWorld, segAngle);

    for(int i = 0; i < numSegmentsToUse; i++) {
    	// clear the marking vectors. Otherwise the old ones stay in there.
    	leftMarkings.clear();
    	midMarkings.clear();
    	rightMarkings.clear();
    	otherMarkings.clear();

    	ROS_INFO("Segment #%u", i+1);

    	// If we did not find a segment at the previous position use one from the road model
    	if(useRoadModelSegment) {
    		if(i < segmentStarts.size()) {
    			segStartWorld = segmentStarts.at(i);
    			segAngle = segmentAngles.at(i);
    		}
    	}

        regions = buildRegions(segStartWorld, segAngle);
		assignLinesToRegions(&regions, unusedLines, leftMarkings, midMarkings, rightMarkings, otherMarkings);
        auto seg = findLaneWithRansac(leftMarkings, midMarkings, rightMarkings, segStartWorld, segAngle);

        // If this is the first segment we can use nullptr as argument for previousSegment
        if(roadModel.segmentFitsToPrevious(&seg, i))
        {
        	seg.creationTimestamp = imgTimestamp;
        	roadModel.updateSegmentAtIndex(seg, i);

        	segAngle = seg.angleTotal; // the current angle of the lane
        	totalSegLength += seg.length;
			segStartWorld = seg.endPositionWorld;
			useRoadModelSegment = false;
        } else {
        	ROS_INFO("Segment does not fit with previous");
        	useRoadModelSegment = true;
        	roadModel.decreaseSegmentTtl(i);
        }

    }

    roadModel.setOdomPointsForSegments();
    roadModel.decreaseAllSegmentTtl();

    // DEBUG
    std::vector<cv::Point2f> worldPts, imgPts;
    std::vector<cv::Scalar> colors;
    std::vector<float> angles;

    roadModel.getSegmentPositions(worldPts, angles, ros::Time(0));
    for(auto s : roadModel.segmentsToDl)  {
        if(s.ttl == 3) {
            colors.push_back(cv::Scalar(0,0,255));
        } else if(s.ttl > 0){
            colors.push_back(cv::Scalar(255));
        } else {
            colors.push_back(cv::Scalar(0,0,0));
        }
    }

    image_operator_.worldToWarpedImg(worldPts, imgPts);

    ROS_INFO("imgPts: %lu, colors: %lu", imgPts.size(), colors.size());
    for(int i = 0; i < imgPts.size(); i++) {
        cv::circle(debugImg_, imgPts.at(i), 10, colors.at(i), 5);
    }
}

///
/// \brief LineDetection::buildRegions
/// \param position Position of the segment in world coordinates
/// \param angle orientation of the segment in [rad]
/// \return the three regions in image coordinates
///
std::vector<cv::RotatedRect> LineDetection::buildRegions(cv::Point2f positionWorld, float angle) {
	int numRegions = 3;
    std::vector<cv::RotatedRect> regions(numRegions);
    std::vector<cv::Point2f> imgPts(1), worldPts(2);
    cv::Vec2f dirVec(cos(angle), sin(angle)); // in world coordinates
    cv::Vec2f leftVec(laneWidthWorld_ * dirVec[1], laneWidthWorld_ * dirVec[0]); // in world coordinates

    // Convert the position to image coordinates
    worldPts.at(0) = positionWorld;
    image_operator_.worldToWarpedImg(worldPts, imgPts);
    cv::Point2f positionImg = imgPts.at(0);

    // Calculate the region size in world coordinates and then convert to size in image coordinates
    cv::Point2f worldPtOuterRegionSize(positionWorld.x + segmentLength_, positionWorld.y + (1.2f * laneWidthWorld_));
    worldPts.at(0) = worldPtOuterRegionSize;
    cv::Point2f worldPtMiddleRegionSize(positionWorld.x + segmentLength_, positionWorld.y + (1.f * laneWidthWorld_));
    worldPts.at(1) = worldPtMiddleRegionSize;
    image_operator_.worldToWarpedImg(worldPts, imgPts);
    cv::Size outerRegionSize(imgPts.at(0).x - positionImg.x, positionImg.y - imgPts.at(0).y);
    cv::Size innerRegionSize(imgPts.at(1).x - positionImg.x, positionImg.y - imgPts.at(1).y);

    // Get the region points in world coordinates
    worldPts.resize(numRegions);
    imgPts.resize(numRegions);

    cv::Point2f centerMidRegion(positionWorld.x + (0.5f * segmentLength_ * dirVec[0]) - (0.5f * leftVec[0]),
         	    				positionWorld.y + (0.5f * segmentLength_ * dirVec[1]) + (0.5f * leftVec[1]));

    cv::Point2f centerRightRegion(centerMidRegion.x + 1.2f*leftVec[0],
    							  centerMidRegion.y - 1.2f*leftVec[1]);

    cv::Point2f centerLeftRegion(centerMidRegion.x - 1.2f*leftVec[0],
    							 centerMidRegion.y + 1.2f*leftVec[1]);


    worldPts.at(0) = centerLeftRegion;
    worldPts.at(1) = centerMidRegion;
    worldPts.at(2) = centerRightRegion;
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    // Create the regions
    regions.at(0) = cv::RotatedRect(imgPts.at(0), outerRegionSize, (-angle * 180.f / M_PI));
    regions.at(1) = cv::RotatedRect(imgPts.at(1), innerRegionSize, (-angle * 180.f / M_PI));
    regions.at(2) = cv::RotatedRect(imgPts.at(2), outerRegionSize, (-angle * 180.f / M_PI));

#ifdef PUBLISH_DEBUG
    if(drawDebugLines_) {
    	for(int i = 0; i < 3; i++) {
    		auto r = regions.at(i);
    		cv::Point2f edges[4];
    		r.points(edges);

    		for(int i = 0; i < 4; i++)
    			cv::line(debugImg_, edges[i], edges[(i+1)%4], cv::Scalar(255));
    	}
    }
#endif

    return regions;
}

///
/// \brief LineDetection::assignLinesToRegions assigns all lines to one of the three regions (left, middle, right)
/// or to others.
/// \param regions three regions (left, middle, right)
/// \param lines the lines which should be assigned to the regions
///
void LineDetection::assignLinesToRegions(std::vector<cv::RotatedRect> *regions, std::vector<Line*> &lines,
                                         std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings,
										 std::vector<Line*> &rightMarkings, std::vector<Line*> &otherMarkings) {

	float orientationAngle = (regions->at(0).angle / 180.f) * M_PI; // the regions angles in [rad]
	if(orientationAngle > M_PI)
		ROS_WARN("Orientation angle > PI");

    for(auto linesIt = lines.begin(); linesIt != lines.end(); ++linesIt) {
    	// first check if the angle is ok to ignore stop and start lines
    	if(fabsf(orientationAngle - (*linesIt)->getAngle()) > M_PI_2) {
    		otherMarkings.push_back(*linesIt);
    	} else {
    		// if angle is ok, test with regions
    		if(lineIsInRegion(*linesIt, &(regions->at(0)), true)) {
    			leftMarkings.push_back(*linesIt);
//    			cv::line(debugImg_, (*linesIt)->iP1_, (*linesIt)->iP2_, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    		} else if(lineIsInRegion(*linesIt, &(regions->at(1)), true)) {
    			midMarkings.push_back(*linesIt);
//    			cv::line(debugImg_, (*linesIt)->iP1_, (*linesIt)->iP2_, cv::Scalar(0, 255), 1, cv::LINE_AA);
    		} else if(lineIsInRegion(*linesIt, &(regions->at(2)), true)) {
    			rightMarkings.push_back(*linesIt);
//    			cv::line(debugImg_, (*linesIt)->iP1_, (*linesIt)->iP2_, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    		} else {
    			otherMarkings.push_back(*linesIt);
    		}
    	}
    }

    // TODO: look over the idea of building rects around the lines again
//    ROS_INFO("---");
    std::vector<cv::Point2f> linePointsWorld;
    for(auto l : leftMarkings) {
    	linePointsWorld.push_back(l->wP1_);
    	linePointsWorld.push_back(l->wP2_);
    }
    if(!linePointsWorld.empty()) {
    	auto rotatedRect = cv::minAreaRect(linePointsWorld);
    	if((rotatedRect.size.height > 0.1f) && (rotatedRect.size.width > 0.1f)) {
//    		ROS_INFO("Left markings rect size = (%.3f, %.3f)", rotatedRect.size.width, rotatedRect.size.height);
//    		leftMarkings.clear();
    	}
    }

    linePointsWorld.clear();
    for(auto l : midMarkings) {
    	linePointsWorld.push_back(l->wP1_);
    	linePointsWorld.push_back(l->wP2_);
    }
    // TODO: if there is a double line, it could be bigger. test if we need a higher limit here
    if(!linePointsWorld.empty()) {
    	auto rotatedRect = cv::minAreaRect(linePointsWorld);
//    	if((rotatedRect.size.height > 0.1f) && (rotatedRect.size.width > 0.1f)) {
//    		ROS_INFO("Mid markings rect size = (%.3f, %.3f)", rotatedRect.size.width, rotatedRect.size.height);
//    	}
    }

    linePointsWorld.clear();
    for(auto l : rightMarkings) {
    	linePointsWorld.push_back(l->wP1_);
    	linePointsWorld.push_back(l->wP2_);
    }
    if(!linePointsWorld.empty()) {
    	auto rotatedRect = cv::minAreaRect(linePointsWorld);
    	if((rotatedRect.size.height > 0.1f) && (rotatedRect.size.width > 0.1f)) {
//    		ROS_INFO("Right markings rect size = (%.3f, %.3f)", rotatedRect.size.width, rotatedRect.size.height);
//    		rightMarkings.clear();
    	}
    }
}

///
/// \brief LineDetection::lineIsInRegion
/// \param line
/// \param region in image coordinates
/// \param isImageCoordinate true, if region is given in image coordinates
/// \return
///
bool LineDetection::lineIsInRegion(Line *line, const cv::RotatedRect *region, bool isImageCoordiante) const {
    cv::Point2f edges[4];
    region->points(edges); // bottomLeft, topLeft, topRight, bottomRight

    if(isImageCoordiante) {
    	// we check if at least on of the lines ends is in the region
    	if(pointIsInRegion(&(line->iP1_), edges))
    		return true;
        else if(pointIsInRegion(&(line->iP2_), edges))
            return true;
        else {
            // in case the line start and end are outside the rect, the line can still intersect with it
            std::vector<cv::Point2f> pts;
            pts.push_back(line->iP1_);
            pts.push_back(line->iP2_);
            auto lineRect = cv::minAreaRect(pts);
            lineRect.size.height += 1;
            lineRect.size.width += 1;
//            ROS_INFO_STREAM("Points " << line->iP1_ << " and " << line->iP2_ << " give Rect at " << lineRect.center << " with size " << lineRect.size);
            int res = cv::rotatedRectangleIntersection(lineRect, *region, pts);
            if(res == cv::INTERSECT_NONE) {
                return false;
            } else {
                return true;
            }
        }

    } else {
    	// we check if at least on of the lines ends is in the region
    	if(pointIsInRegion(&(line->wP1_), edges))
    		return true;
    	else
    		return pointIsInRegion(&(line->wP2_), edges);
    }
}

bool LineDetection::pointIsInRegion(cv::Point2f *pt, cv::Point2f *edges) const {
    auto u = edges[0] - edges[1];
    auto v = edges[0] - edges[3];

    auto uDotPt = u.dot(*pt);
    auto uDotP1 = u.dot(edges[0]);
    auto uDotP2 = u.dot(edges[1]);
    auto vDotPt = v.dot(*pt);
    auto vDotP1 = v.dot(edges[0]);
    auto vDotP4 = v.dot(edges[3]);

    bool xOk = (uDotPt < uDotP1 && uDotPt > uDotP2) ||
               (uDotPt > uDotP1 && uDotPt < uDotP2);
    bool yOk = (vDotPt < vDotP1 && vDotPt > vDotP4) ||
               (vDotPt > vDotP1 && vDotPt < vDotP4);

    return xOk && yOk;
}

///
/// \brief LineDetection::findLaneWithRansac
/// \param leftMarkings
/// \param midMarkings
/// \param rightMarkings
/// \param segStartWorld the segments position in world coordinates
/// \param prevAngle
/// \return
///
Segment LineDetection::findLaneWithRansac(std::vector<Line*> &leftMarkings,
										  std::vector<Line*> &midMarkings,
										  std::vector<Line*> &rightMarkings,
                                          cv::Point2f segStartWorld, float prevAngle) {
    float bestAngle = prevAngle;
    float bestScore = 0.f;
    Line *bestLine;
    cv::Point2f laneMidPt = segStartWorld;
    cv::Point2f bestLeft, bestMid, bestRight;
    float detectedRange = 0.f;
    int numLaneMarkingsDetected = 0;
    size_t numLines = leftMarkings.size() + midMarkings.size() + rightMarkings.size();
    size_t iteration = 0;
    std::vector<cv::Point2f> worldPts, imgPts;
    worldPts.push_back(segStartWorld);
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    if(numLines == 0) {
        ROS_WARN("No lines for Ransac");
        return Segment(segStartWorld, imgPts.at(0), 0.f, prevAngle, segmentLength_, 0.f);
    }

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, numLines - 1);

    ROS_INFO("---- RANSAC ----");
//    ROS_INFO_STREAM("Left markings: " << leftMarkings.size());
//    ROS_INFO_STREAM("Mid markings: " << midMarkings.size());
//    ROS_INFO_STREAM("Right markings: " << rightMarkings.size());

    while(bestScore < 0.9 && (iteration++ < maxRansacInterations_)) {
        // Select a random line and get its angle
    	// Also get a random point and move it on the expected middle line
        int randomIdx = distribution(generator);
        float currentAngle = prevAngle;
        Line *currentLine;

        // 1) Select a random line
        if(randomIdx < leftMarkings.size()) {
        	currentLine = leftMarkings.at(randomIdx);
        } else if((randomIdx - leftMarkings.size()) < midMarkings.size()) {
        	currentLine = midMarkings.at(randomIdx - leftMarkings.size());
        } else {
        	currentLine = rightMarkings.at(randomIdx - leftMarkings.size() - midMarkings.size());
        }

        currentAngle = currentLine->getAngle();

        // 2) Test how many lines have about the same angle and their distance fits
        int numInliers = 0;

        std::function<bool (Line*)> lineCmpFunc = [this, currentAngle, currentLine](Line *l) {
        	if(fabsf(l->getAngle() - currentAngle) < (5.f / 180.f * M_PI)) {
        		// angle diff should be < 3 [degree]
        		float distanceToLine = distanceBetweenLines(*l, *currentLine);

        		if(distanceToLine < laneVar_) {
        			return true;
        		}

        		if(fabsf(distanceToLine - laneWidthWorld_) < laneVar_) {
        			return true;
        		}

        		if(fabsf(distanceToLine - 2*laneWidthWorld_) < laneVar_) {
        			return true;
        		}

        		return false;
        	}
        };

        numInliers += std::count_if(leftMarkings.begin(), leftMarkings.end(), lineCmpFunc);
        numInliers += std::count_if(midMarkings.begin(), midMarkings.end(), lineCmpFunc);
        numInliers += std::count_if(rightMarkings.begin(), rightMarkings.end(), lineCmpFunc);

        // 3) Compute score
        float newScore = static_cast<float>(numInliers) / static_cast<float>(numLines);
        if(newScore > bestScore) {
            bestScore = newScore;
            bestAngle = currentAngle;
            bestLine = currentLine;
        }
    }

    // 4) Collect points from every lane marking
    std::function<bool (Line*)> inlierFunc = [this, bestAngle, bestLine](Line *l) {
    	if(fabsf(l->getAngle() - bestAngle) < (5.f / 180.f * M_PI)) {
    		// angle diff should be < 3 [degree]
    		float distanceToLine = distanceBetweenLines(*l, *bestLine);

    		if(distanceToLine < laneVar_) {
    			return true;
    		}

    		if(fabsf(distanceToLine - laneWidthWorld_) < laneVar_) {
    			return true;
    		}

    		if(fabsf(distanceToLine - 2*laneWidthWorld_) < laneVar_) {
    			return true;
    		}

    		return false;
    	}
    };

    std::vector<cv::Point2f> leftInlierPtsWorld, midInlierPtsWorld, rightInlierPtsWorld;

    for(auto l : midMarkings) {
    	if(inlierFunc(l)) {
//    		cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,255), 3, cv::LINE_AA);
    		midInlierPtsWorld.push_back(l->wP1_);
    		midInlierPtsWorld.push_back(l->wP2_);
    	}
    }

    for(auto l : leftMarkings) {
    	if(inlierFunc(l)) {
//    		cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    		leftInlierPtsWorld.push_back(l->wP1_);
    		leftInlierPtsWorld.push_back(l->wP2_);
    	}
    }

    for(auto l : rightMarkings) {
    	if(inlierFunc(l)) {
//    		cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    		rightInlierPtsWorld.push_back(l->wP1_);
    		rightInlierPtsWorld.push_back(l->wP2_);
    	}
    }

//    ROS_INFO("Left inliers: %lu", leftInlierPtsWorld.size() / 2);
//    ROS_INFO("Mid inliers: %lu", midInlierPtsWorld.size() / 2);
//    ROS_INFO("Right inliers: %lu", rightInlierPtsWorld.size() / 2);

    // 5) Fit polynom in each line
    int lanePosFrom = 0; // DEBUG
    if(!leftInlierPtsWorld.empty()) {
    	Polynom poly(1, leftInlierPtsWorld);

    	worldPts.clear();
    	imgPts.clear();

    	auto minElem = std::min_element(leftInlierPtsWorld.begin(), leftInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});
    	auto maxElem = std::max_element(leftInlierPtsWorld.begin(), leftInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});

    	float xMin = minElem->x;
    	float xMax = maxElem->x;

    	// Set lane position
    	laneMidPt = cv::Point2f(segStartWorld.x, poly.atX(segStartWorld.x));
    	laneMidPt.y -= laneWidthWorld_ * 1.5f;
    	lanePosFrom = 1;

    	detectedRange += xMax;
    	numLaneMarkingsDetected++;

//    	for(float x = xMin; x < xMax; x += 0.2f) {
//    		worldPts.push_back(cv::Point2f(x, poly.atX(x)));
//    	}
//    	worldPts.push_back(cv::Point2f(xMax, poly.atX(xMax)));

//    	image_operator_.worldToWarpedImg(worldPts, imgPts);

//    	for(auto p : imgPts) {
//    		cv::circle(debugImg_, p, 4, cv::Scalar(0,0,255), 2);
//    	}
    }

    if(!rightInlierPtsWorld.empty()) {
    	Polynom poly(1, rightInlierPtsWorld);

    	worldPts.clear();
    	imgPts.clear();

    	auto minElem = std::min_element(rightInlierPtsWorld.begin(), rightInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});
    	auto maxElem = std::max_element(rightInlierPtsWorld.begin(), rightInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});

    	float xMin = minElem->x;
    	float xMax = maxElem->x;

    	if((xMax - xMin) < 0.1f) {
    		ROS_WARN("right line length = %.3f", (xMax - xMin));
    	}

    	// Set lane position
    	laneMidPt = cv::Point2f(segStartWorld.x, poly.atX(segStartWorld.x));
    	laneMidPt.y += laneWidthWorld_ * .5f;
    	lanePosFrom = 3;

    	detectedRange += xMax;
    	numLaneMarkingsDetected++;

//    	for(float x = xMin; x < xMax; x += 0.2f) {
//    		worldPts.push_back(cv::Point2f(x, poly.atX(x)));
//    	}
//    	worldPts.push_back(cv::Point2f(xMax, poly.atX(xMax)));

//    	image_operator_.worldToWarpedImg(worldPts, imgPts);

//    	for(auto p : imgPts) {
//    		cv::circle(debugImg_, p, 4, cv::Scalar(0,0,255), 2);
//    	}
    }

    if(!midInlierPtsWorld.empty()) {
    	Polynom poly(1, midInlierPtsWorld);

    	worldPts.clear();
    	imgPts.clear();

    	auto minElem = std::min_element(midInlierPtsWorld.begin(), midInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});
    	auto maxElem = std::max_element(midInlierPtsWorld.begin(), midInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});

    	float xMin = minElem->x;
    	float xMax = maxElem->x;

    	if((xMax - xMin) < 0.1f) {
    		ROS_WARN("mid line length = %.3f", (xMax - xMin));
    	}

    	// Set lane position
		laneMidPt = cv::Point2f(segStartWorld.x, poly.atX(segStartWorld.x));
		laneMidPt.y -= laneWidthWorld_ * .5f;
		lanePosFrom = 2;

		detectedRange += xMax;
		numLaneMarkingsDetected++;

//		for(float x = xMin; x < xMax; x += 0.2f) {
//			worldPts.push_back(cv::Point2f(x, poly.atX(x)));
//		}
//		worldPts.push_back(cv::Point2f(xMax, poly.atX(xMax)));

//		image_operator_.worldToWarpedImg(worldPts, imgPts);

//		for(auto p : imgPts) {
//			cv::circle(debugImg_, p, 4, cv::Scalar(0,255), 2);
//		}
    }

    detectedRange /= numLaneMarkingsDetected;
    float segmentLen = std::min(detectedRange - segStartWorld.x, segmentLength_);
    cv::Point2f segEndWorld;
    segEndWorld.x = laneMidPt.x + cos(bestAngle) * segmentLen;
    segEndWorld.y = laneMidPt.y + sin(bestAngle) * segmentLen;

    // Convert all needed points to image coordinates
    worldPts.clear();
    imgPts.clear();
    worldPts.push_back(laneMidPt); // idx 0
    worldPts.push_back(bestLeft); // idx 1
    worldPts.push_back(bestMid); // idx 2
    worldPts.push_back(bestRight); // idx 3
    worldPts.push_back(segEndWorld); // idx 4
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    // Create the segment
    Segment s(laneMidPt, imgPts.at(0), prevAngle - bestAngle, bestAngle, segmentLen, bestScore);
    s.leftPosW = bestLeft;
    s.midPosW = bestMid;
    s.rightPosW = bestRight;
    s.leftPosI = imgPts.at(1);
    s.midPosI = imgPts.at(2);
    s.rightPosI = imgPts.at(3);
    s.endPositionImage = imgPts.at(4);
    s.endPositionWorld = segEndWorld;

    ROS_INFO("  Lane position built from %s line", ((lanePosFrom == 1) ? "left" : ((lanePosFrom == 2) ? "mid" : "right")));

//    ROS_INFO("Orig segmentStart: (%.2f, %.2f)", segStartWorld.x, segStartWorld.y);
//    ROS_INFO("New lane mid: (%.2f, %.2f)", laneMidPt.x, laneMidPt.y);
//    ROS_INFO("Detected range = %.2f, segment length = %.3f", detectedRange, (detectedRange - segStartWorld.x));

    return s;
}

bool LineDetection::findIntersection(Segment &resultingSegment, float segmentAngle, cv::Point2f segStartWorld,
		std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings, std::vector<Line*> &rightMarkings) {

	bool foundIntersection = false;
	bool middleExists = false, rightExists = false, leftExists = false;
	float stopLineAngle = 0.f;
	float stopLineXpos = 0.f;
	int numStopLines = 0;

	for(auto l : leftMarkings) {
		if(std::abs(l->getAngle() - segmentAngle - M_PI_2) < (M_PI  / 7.0f)) {
			leftExists = true;
			cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(155), 2, cv::LINE_AA);
		}
	}

	for(auto l : midMarkings) {
		if(std::abs(l->getAngle() - segmentAngle - M_PI_2) < (M_PI  / 7.0f)) {
			middleExists = true;
			numStopLines++;
			stopLineAngle += l->getAngle();
			stopLineXpos += l->wP1_.x;
			stopLineXpos += l->wP2_.x;
			cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(57,189,37), 2, cv::LINE_AA);
		}
	}

	for(auto l : rightMarkings) {
		if(std::abs(l->getAngle() - segmentAngle - M_PI_2) < (M_PI  / 7.0f)) {
			rightExists = true;
			cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,255,255), 2, cv::LINE_AA);
		}
	}

	// Build a segment for the whole intersection
	Eigen::Vector2f laneDir;
	SegmentType intersectionType;
	float drivingDirectionAngle = segmentAngle;
	if(middleExists && (leftExists || rightExists)) {
//		ROS_INFO("Intersection found - stop");
		foundIntersection = true;
		intersectionType = SegmentType::INTERSECTION_STOP;
		stopLineAngle = stopLineAngle / static_cast<float>(numStopLines);
		drivingDirectionAngle = stopLineAngle - M_PI_2;
//		laneDir = Eigen::Vector2f(sin(stopLineAngle), - cos(stopLineAngle)); // direction of vector perpendicular to stop line
		laneDir = Eigen::Vector2f(cos(drivingDirectionAngle), sin(drivingDirectionAngle));
	}
	if(!middleExists && leftExists && rightExists) {
//		ROS_INFO("Intersection found - do not stop");
		foundIntersection = true;
		intersectionType = SegmentType::INTERSECTION_GO_STRAIGHT;
		laneDir = Eigen::Vector2f(cos(segmentAngle), sin(segmentAngle));
	}

	if(foundIntersection) {
		std::vector<cv::Point2f> imgPts, worldPts;
		float segmentLength = laneWidthWorld_ * 2.5f;

		worldPts.push_back(segStartWorld);
		image_operator_.worldToWarpedImg(worldPts, imgPts);
		// Segment(cv::Point2f worldPos, cv::Point2f imagePos, float angleDiff_, float angleTotal_, float len, float prob)
		resultingSegment = Segment(segStartWorld, imgPts.at(0), segmentAngle - drivingDirectionAngle,
				drivingDirectionAngle, segmentLength, 1.0f);
		resultingSegment.segmentType = intersectionType;

		// TEST FOR POSITION
		cv::Point2f intersectionPos(stopLineXpos * .5, segStartWorld.y);

		worldPts.resize(4);
		imgPts.clear();

		worldPts.at(0) = cv::Point2f(intersectionPos.x, intersectionPos.y - 0.5f*laneWidthWorld_);
		worldPts.at(1) = cv::Point2f(intersectionPos.x, intersectionPos.y + 1.5f*laneWidthWorld_);
		worldPts.at(2) = cv::Point2f(intersectionPos.x + 2.0f*laneWidthWorld_, intersectionPos.y + 1.5f*laneWidthWorld_);
		worldPts.at(3) = cv::Point2f(intersectionPos.x + 2.0f*laneWidthWorld_, intersectionPos.y - 0.5f*laneWidthWorld_);
		image_operator_.worldToWarpedImg(worldPts, imgPts);

		for(int i = 0; i < 4; i++) {
			cv::line(debugImg_, imgPts[i], imgPts[(i+1)%4], cv::Scalar(0,255));
		}
	}

	return foundIntersection;
}

//cv::Point2f LineDetection::findTrajectoryPoint(std::vector<tf::Stamped<tf::Point>> &drivingLine) {
//	if(drivingLine.empty()) {
//		return cv::Point2f(1.f, 0.f);
//	}
//
//	// TEST POLYFIT
//	int order = 4;
//	cv::Mat srcX(drivingLine.size(), 1, CV_32FC1);
//	cv::Mat srcY(drivingLine.size(), 1, CV_32FC1);
//	cv::Mat dst(order+1, 1, CV_32FC1);
//
//	for(int i = 0; i < drivingLine.size(); i++) {
//		srcX.at<float>(i, 0) = drivingLine.at(i).x();
//		srcY.at<float>(i, 0) = drivingLine.at(i).y();
//	}
//
////	ROS_INFO_STREAM("srcX dims = " << srcX.size << "; srcY dims = " << srcY.size << "; dst dims = " << dst.size);
//	polyfit(srcX, srcY, dst, order);
////	ROS_INFO_STREAM("PolyFit = " << dst);
//
//	std::vector<cv::Point2f> imagePoints, worldPoints;
//
//	ROS_INFO("----------------------------");
//	if(prevPolyCoeff.rows == (order+1)) {
//		auto a = prevPolyCoeff.at<float>(0,0);
//		auto b = prevPolyCoeff.at<float>(1,0);
//		auto c = prevPolyCoeff.at<float>(2,0);
//		auto d = prevPolyCoeff.at<float>(3,0);
//		auto e = prevPolyCoeff.at<float>(4,0);
//
//		for(int i = 2; i < 10; i++) {
//			auto x = i * 0.2;
//			worldPoints.push_back(cv::Point2f(x, e*x*x*x*x + d*x*x*x + c*x*x + b*x + a));
//		}
//
//	}
//
//	prevPolyCoeff = dst;
//
//	auto a = dst.at<float>(0,0);
//	auto b = dst.at<float>(1,0);
//	auto c = dst.at<float>(2,0);
//	auto d = dst.at<float>(3,0);
//	auto e = dst.at<float>(4,0);
//
//	for(int i = 2; i < 10; i++) {
//		auto x = i * 0.2;
//		worldPoints.push_back(cv::Point2f(x, e*x*x*x*x + d*x*x*x + c*x*x + b*x + a));
//	}
//
//	// COMPARE THE POLYNOMS
//	float sqrdError = 0.f;
//	for(int i = 0; i < worldPoints.size() / 2; i++) {
//		auto xDiff = worldPoints.at(i).x - worldPoints.at(i + worldPoints.size()/2).x;
//		auto yDiff = worldPoints.at(i).y - worldPoints.at(i + worldPoints.size()/2).y;
//		sqrdError += xDiff*xDiff + yDiff*yDiff;
//	}
//
//	ROS_INFO("Error = %f", sqrt(sqrdError));
//
//	image_operator_.worldToWarpedImg(worldPoints, imagePoints);
//	for(int i = 0; i < imagePoints.size(); i++) {
////	for(auto p : imagePoints) {
//		cv::circle(debugImg_, imagePoints.at(i), 5,
//				(i<8) ? cv::Scalar(0,0,255) : cv::Scalar(0,255),
//				2, cv::LINE_AA);
//	}
//
//
//	float x = trajectoryDist;
//	return cv::Point2f(x, e*x*x*x*x + d*x*x*x + c*x*x + b*x + a);
//}

///
/// \brief LineDetection::findLinesWithHough
/// Extract Lines from the image using Hough Lines
/// \param img A CvImagePtr to the current image where we want to search for lines
/// \param houghLines A std::vector where the Lines will be returned. Has to be empty.
///
void LineDetection::findLinesWithHough(cv::Mat &img, std::vector<Line> &houghLines) {
    cv::Mat processingImg;

    // Blur the image and find edges with Canny
    cv::GaussianBlur(img, processingImg, cv::Size(15, 15), 0, 0);
    cv::Canny(processingImg, processingImg, cannyThreshold_, cannyThreshold_ * 3, 3);


    // Get houghlines
    std::vector<cv::Vec4i> hLinePoints;
    cv::HoughLinesP(processingImg, hLinePoints, 1, CV_PI / 180, houghThresold_, houghMinLineLen_, houghMaxLineGap_);

    // Extract points for houghLines and convert to world-coordinates
    std::vector<cv::Point2f> imagePoints, worldPoints;
    for(size_t i = 0; i < hLinePoints.size(); i++) {
        cv::Vec4i currentPoints = hLinePoints.at(i);
        // Ignore lines very close to the car and about to be parallel to image y-axis
        if(!((currentPoints[0] < 50) && (currentPoints[2] < 50))) {
            imagePoints.push_back(cv::Point(currentPoints[0], currentPoints[1]));
            imagePoints.push_back(cv::Point(currentPoints[2], currentPoints[3]));
        }
    }

    if (imagePoints.size() == 0) {
        ROS_WARN_STREAM("No hough lines found in image");
        return;
    }
    image_operator_.warpedImgToWorld(imagePoints, worldPoints);

    // Build lines from points
    std::vector<Line> lines;
    for(size_t i = 0; i < worldPoints.size(); i += 2) {
        houghLines.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
    }

#if 0
    // Split lines which are longer than segmentLength (world coordinates)
    worldPoints.clear();
    imagePoints.clear();
    for(auto it = lines.begin(); it != lines.end(); ++it) {
    	int splitInto = static_cast<int>(it->getWorldLength() / segmentLength_) + 1;
//    	ROS_INFO_STREAM("Length = " << it->getWorldLength() << " split into " << splitInto);

    	if(splitInto > 1) {
    		// Build normalized vector
    		cv::Vec2f dir = it->wP2_ - it->wP1_;
    		auto length = sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
    		dir[0] = dir[0] / length;
    		dir[1] = dir[1] / length;

    		cv::Point2f curPos = it->wP1_;
    		for(int i = 0; i < (splitInto - 1); i++) {
    			worldPoints.push_back(curPos);
    			curPos.x += dir[0] * segmentLength_;
    			curPos.y += dir[1] * segmentLength_;
    			worldPoints.push_back(curPos);
    		}
    		// This is the last part of the split line
    		worldPoints.push_back(curPos);
    		worldPoints.push_back(it->wP2_);
    	} else {
    		houghLines.push_back(*it);
    	}
    }

    // Build split lines
    image_operator_.worldToWarpedImg(worldPoints, imagePoints);
    for(size_t i = 0; i < worldPoints.size(); i += 2) {
    	houghLines.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
    }

#endif
}

/*
 * In world coordinates
 */
float LineDetection::pointToLineDistance(Line &l, const cv::Point2f &p) {
    // http://paulbourke.net/geometry/pointlineplane/

    if(getDistanceBetweenPoints(l.wP1_, l.wP2_) == 0) {
        ROS_WARN("Line with length 0");
        return std::numeric_limits<float>::max();
    }

    float u = ((p.x - l.wP1_.x)*(l.wP2_.x - l.wP1_.x) + (p.y - l.wP1_.y)*(l.wP2_.y - l.wP1_.y)) /
            (getDistanceBetweenPoints(l.wP1_, l.wP2_));

    float intersectionPointX = l.wP1_.x + u*(l.wP2_.x - l.wP1_.x);
    float intersectionPointY = l.wP1_.y + u*(l.wP2_.y - l.wP1_.y);

    return getDistanceBetweenPoints(p, cv::Point2f(intersectionPointX, intersectionPointY));
}

float LineDetection::distanceBetweenLines(Line &a, Line &b) {
    return	fminf(pointToLineDistance(a, b.wP1_),
                  fminf(pointToLineDistance(a, b.wP2_),
                        fminf(pointToLineDistance(b, a.wP1_), pointToLineDistance(b, a.wP2_))));
}

///
/// \brief LineDetection::reconfigureCB
/// Used by the dynamic reconfigure server
/// \param config
/// \param level
///
void LineDetection::reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level) {
    image_operator_.setConfig(config);
    drawDebugLines_ = config.draw_debug;
    laneWidthWorld_ = config.lane_width;
    laneVar_ = config.lane_var;
    lineAngle_ = config.lineAngle;
    maxSenseRange_ = config.maxSenseRange;
    cannyThreshold_ = config.cannyThreshold;
    houghThresold_ = config.houghThreshold;
    houghMinLineLen_ = config.houghMinLineLen;
    houghMaxLineGap_ = config.houghMaxLineGap;
    segmentLength_ = config.segmentLength;
    maxRansacInterations_ = config.ransacIterations;

    roadModel.setLaneWidth(laneWidthWorld_);
    roadModel.setDefaultPolyOrder(config.poly_order);
    roadModel.setMaxPolyErrorThresh(config.poly_error_thresh);
}

} // namespace drive_ros_image_recognition
