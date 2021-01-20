/**
 * @brief filter class for two dimensional point clouds in polar coordinates
 * (LIDAR readout filter) THE POINTCLOUD MUST BE CLOSED (FULL CIRCLE)
 * @author created by Julius Kramer on 01.01.21
 * @copyright 2021 Julius Kramer julius.kramer@students.fhv.at
 */

#ifndef FILTER_HPP
#define FILTER_HPP

#include <cmath>
#include <list>
#define PI 3.14159265359

typedef struct Point_P_t {
  /**
   * @param distance unit: mm
   * @param angle unit: degrees
   */
  double distance;
  double angle;
} Point_Polar;

typedef struct Point_K_t {
  /**
   * @param x unit: mm
   * @param y unit: mm
   */
  double x;
  double y;
} Point_Cartesian;

template <typename TYPE> class RollingBuffer {
public:
  /**
   * @brief this class provides a rolling buffer for circular datasets
   * @param maxSize value at which the oldest elements will be removed
   */
  RollingBuffer(const uint32_t maxSize) : m_maxSize(maxSize) {}

  void add(TYPE point) {
    /**
     * @brief add a single point to the dataset, if the size increases m_size,
     * the last item will be discarded
     */
    m_buffer.push_front(point);
    if (m_buffer.size() > m_maxSize) {
      m_buffer.pop_back();
    }
  }

  TYPE average() {
    /**
     * @brief calculates the average over the whole dataset
     */
    TYPE average = 0;
    for (TYPE point : m_buffer) {
      average += point;
    }
    return (average /= ((TYPE)m_buffer.size()));
  }

private:
  const uint32_t m_maxSize;
  std::list<TYPE> m_buffer;
};

class Calc {
  /**
   * @brief various functions to do calculations on std::list datatypes,
   * primarily for types "Point_Polar" and "Point_Cartesian"
   */
public:
  Calc() {}

  std::list<Point_Polar> getPolar(std::list<Point_Cartesian> cartesian) {
    /**
     * @brief converts a dataset given in cartesian coordinates into a dataset
     * in polar coordiantes
     */
    std::list<Point_Polar> PolarVals;
    for (Point_Cartesian point : cartesian) {
      Point_Polar converted;
      converted.angle = (atan(point.y / point.x) * 180) / PI;
      if ((point.y < 0) && (point.x < 0)) {
        converted.angle += 180;
      } else if (point.y < 0) {
        converted.angle += 360.0;
      } else if (point.x < 0) {
        converted.angle += 180;
      }
      converted.distance = sqrt((point.x * point.x) + (point.y * point.y));
      PolarVals.push_back(converted);
    }
    return PolarVals;
  }

  std::list<Point_Cartesian> getCartesian(std::list<Point_Polar> polar) {
    /**
     * @brief converts a dataset given in polar coordinates into a dataset in
     * cartesian coordiantes
     */
    std::list<Point_Cartesian> CartesianVals;
    for (Point_Polar point : polar) {
      Point_Cartesian converted;
      converted.x = cos(point.angle * (PI / 180.0)) * point.distance;
      converted.y = sin(point.angle * (PI / 180.0)) * point.distance;
      CartesianVals.push_back(converted);
    }
    return CartesianVals;
  }

  std::list<double> getAngles(std::list<Point_Polar> polar) {
    /**
     * @brief extracts individual angle-values dataset from a combined
     * (distance,angle) dataset
     */
    std::list<double> angleVals;
    for (Point_Polar point : polar) {
      angleVals.push_back(point.angle);
    }
    return angleVals;
  }

  std::list<double> getValuesX(std::list<Point_Cartesian> input) {
    /**
     * @brief extracts individual x-values dataset from a combined (x,y) dataset
     */
    std::list<double> xVals;
    for (Point_Cartesian point : input) {
      xVals.push_back(point.x);
    }
    return xVals;
  }

  std::list<double> getValuesY(std::list<Point_Cartesian> input) {
    /**
     * @brief extracts individual y-values dataset from a combined (x,y) dataset
     */
    std::list<double> yVals;
    for (Point_Cartesian point : input) {
      yVals.push_back(point.y);
    }
    return yVals;
  }

  std::list<Point_Cartesian> combine(std::list<double> x, std::list<double> y) {
    /**
     * @brief combines individual x, y datasets into one (x,y) dataset
     */
    std::list<Point_Cartesian> combinedVals;
    uint32_t length = (uint32_t)(x.size() >= y.size() ? x.size() : y.size());
    for (uint32_t i = 0; i < length; i += 1) {
      Point_Cartesian point;
      point.x = x.front();
      point.y = y.front();
      combinedVals.push_back(point);
      x.pop_front();
      y.pop_front();
    }
    return combinedVals;
  }

  std::list<Point_Cartesian> derivation(std::list<Point_Cartesian> input) {
    /**
     * @brief calculates the derivative of the given dataset
     */
    std::list<Point_Cartesian> derivationVals;
    Point_Cartesian previousPoint = input.back();
    for (Point_Cartesian point : input) {
      Point_Cartesian currentderivation;
      currentderivation.x = point.x;
      currentderivation.y =
          (point.y - previousPoint.y) / (point.x - previousPoint.x);
      derivationVals.push_back(currentderivation);
      previousPoint = point;
    }
    return derivationVals;
  }

  std::list<bool> thresholdMask(std::list<double> input, double threshold) {
    /**
     * @brief created a mask containing true/false to indicate if the value at
     * the current position is greather than or equal than the desired threshold
     */
    std::list<bool> mask;
    for (double point : input) {
      mask.push_back(abs((int)point) >= threshold);
    }
    return mask;
  }

  std::list<double> maskDataset(std::list<double> input, std::list<bool> mask) {
    /**
     * @brief manipulates a dataset in the following way:
     * - keeps the data if mask at specific position is true
     * - overwrites the data with the last valid data if mask is false ("trails
     * last valid data through the next dataslots until mask is true again")
     */
    std::list<double> masked;
    double lastValidValue = 0;
    uint32_t countInvalidValuesAtBeginning = 0;
    bool validValueFoundOnce = false;
    for (double point : input) {
      if (mask.front() == true) {
        validValueFoundOnce = true;
        masked.push_back(point);
        lastValidValue = point;
      } else {
        if (!validValueFoundOnce) {
          countInvalidValuesAtBeginning += 1;
        } else {
          masked.push_back(lastValidValue);
        }
      }
      mask.pop_front();
    }

    for (uint32_t i = 0; i < countInvalidValuesAtBeginning; i += 1) {
      masked.push_front(lastValidValue);
    }

    return masked;
  }

  std::list<double> shiftDatasetBy(std::list<double> points,
                                   int32_t shiftPositions) {
    /**
     * @brief shifts / rolls a whole dataset (rolling) by a defined number in
     * each direction
     */
    if (shiftPositions > 0) {
      /**
       * @note Shift whole list by "shiftPositions" to the back
       */
      for (uint32_t index = 0; index < shiftPositions; index += 1) {
        points.push_front(points.back());
        points.pop_back();
      }
    } else {
      /**
       * @note Shift whole list by "shiftPositions" to the front
       */
      for (uint32_t index = 0; index < abs(shiftPositions); index += 1) {
        points.push_back(points.front());
        points.pop_front();
      }
    }
    return points;
  }

  std::list<double> movingAverageAdjusted(std::list<double> points,
                                          uint32_t period) {
    /**
     * @brief calculates a moving average (rolling) and shifts it by period/2 so
     * it does not tail or lead the origial curve
     */

    RollingBuffer<double> buffer(period);
    std::list<double> average_adj;
    std::list<Point_Cartesian> returnList;

    /**
     * @note Calc Average from position "period" to "end"
     */
    uint32_t index = 0;
    for (double point : points) {
      buffer.add(point);
      index += 1;
      if (index >= period) {
        average_adj.push_back(buffer.average());
      }
    }

    /**
     * @note Calc Average from position "begin" to "period"
     */
    index = 0;
    for (double point : points) {
      buffer.add(point);
      index += 1;
      if (index >= period) {
        break;
      }
      average_adj.push_back(buffer.average());
    }

    /**
     * @note Shift whole list by "period / 2"
     */
    for (index = 0; index < (period / 2); index += 1) {
      average_adj.push_front(average_adj.back());
      average_adj.pop_back();
    }

    return average_adj;
  }
};

class Filter {
public:
  /**
   * @brief filter class for filtering of lidar data (circular dataset, ordered)
   * @param movingAveragePeriod set the period for general smoothing, should be
   * an EVEN VALUE, to prevent unwanted output distortion, a good value is
   * approximately ONE FIFTEENTH of the size of the dataset
   * @param filterThreshold set the threshold to filter out unwanted datapoints
   * and align the dataset parallel to the coordinate axes
   */
  Filter(uint32_t movingAveragePeriod = 10, uint32_t filterThreshold = 5)
      : m_movingAveragePeriod(movingAveragePeriod),
        m_threshold(filterThreshold) {}

  void updateSettings(uint32_t movingAveragePeriod, uint32_t filterThreshold) {
    /**
     * @brief update filter parameters, will take action when the next filtering
     * is requested
     */
    m_movingAveragePeriod = movingAveragePeriod;
    m_threshold = filterThreshold;
  }

  uint32_t getMovingAveragePeriod() {
    /**
     * @returns the currently set moving average period value
     */
    return m_movingAveragePeriod;
  }

  uint32_t getThreshold() {
    /**
     * @returns the currently set threshold value
     */
    return m_threshold;
  }

  std::list<Point_Polar> filterDataset(std::list<Point_Polar> input) {
    /**
     * @param input ordered dataset in polar coordinates
     * @returns ordererd filtered dataset in polar coordinates
     */
    m_polarPointCloud = input;
    m_CartesianPointCloud = Calc().getCartesian(m_polarPointCloud);

    std::list<double> values_angle = Calc().getAngles(m_polarPointCloud);
    std::list<double> values_x = Calc().getValuesX(m_CartesianPointCloud);
    std::list<double> values_y = Calc().getValuesY(m_CartesianPointCloud);

    /**
     * @note Calculate moving averages for both x and y lists
     */
    std::list<double> averageX =
        Calc().movingAverageAdjusted(values_x, m_movingAveragePeriod);
    std::list<double> averageY =
        Calc().movingAverageAdjusted(values_y, m_movingAveragePeriod);

    /**
     * @note Make plotdata for x and y: x value in relation to angle and y value
     * in relation to angle
     */
    std::list<Point_Cartesian> plotdataX =
        Calc().combine(values_angle, averageX);
    std::list<Point_Cartesian> plotdataY =
        Calc().combine(values_angle, averageY);

    /**
     * @note Calculate derivation for both sets of plotdata
     */
    std::list<Point_Cartesian> derivationX = Calc().derivation(plotdataX);
    std::list<Point_Cartesian> derivationY = Calc().derivation(plotdataY);

    /**
     * @note Calculate moving average for both derivations x and y
     */
    std::list<double> averagederivationX = Calc().movingAverageAdjusted(
        Calc().getValuesY(derivationX),
        m_movingAveragePeriod); // getValuesY is correct here
    std::list<double> averagederivationY = Calc().movingAverageAdjusted(
        Calc().getValuesY(derivationY),
        m_movingAveragePeriod); // getValuesY is also correct here

    /**
     * @note Calculate threshold of both x and y derivations
     */
    std::list<bool> maskX =
        Calc().thresholdMask(averagederivationX, m_threshold);
    std::list<bool> maskY =
        Calc().thresholdMask(averagederivationY, m_threshold);

    /**
     * @note Shift dataset by period/2 to fix shifting error due to previous
     * operations
     */
    averageX = Calc().shiftDatasetBy(averageX, -(m_movingAveragePeriod / 2));
    averageY = Calc().shiftDatasetBy(averageY, -(m_movingAveragePeriod / 2));

    /**
     * @note Apply threshold masks to averaged original values
     */
    std::list<double> filteredX = Calc().maskDataset(averageX, maskX);
    std::list<double> filteredY = Calc().maskDataset(averageY, maskY);

    /**
     * @note Combine filtered data
     */
    std::list<Point_Cartesian> filteredPointCloud =
        Calc().combine(filteredX, filteredY);

    /**
     * @note Convert back to polar
     */
    std::list<Point_Polar> filteredPolar = Calc().getPolar(filteredPointCloud);

    return filteredPolar;
  }

private:
  std::list<Point_Polar> m_polarPointCloud;
  std::list<Point_Cartesian> m_CartesianPointCloud;
  uint32_t m_movingAveragePeriod; // Should be an even value
  uint32_t m_threshold;
};

#endif
