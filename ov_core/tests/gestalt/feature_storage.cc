// Exercise the actual database implementation with out-of-order retry history.
#include "feat/FeatureDatabase.h"
#include "feat/Feature.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

static void add(const std::shared_ptr<ov_core::FeatureDatabase> &db,
                double time, float payload, size_t camera = 0) {
  db->update_feature(7, time, camera, payload, payload + 1, payload + 2, payload + 3);
}

int main() try {
  auto destination = std::make_shared<ov_core::FeatureDatabase>();
  auto source = std::make_shared<ov_core::FeatureDatabase>();
  add(destination, 3, 30);
  add(destination, 1, 10);
  add(source, 1, 99); // Existing destination wins; never overwrite its pixels.
  add(source, 2, 20);
  add(source, 2, 21); // Preserve the upstream source-duplicate behavior.
  add(source, 4, 40, 1);
  destination->append_new_measurements(source);
  auto feature = destination->get_feature(7);
  if (feature->timestamps.at(0) != std::vector<double>{3, 1, 2, 2} ||
      feature->uvs.at(0)[1](0) != 10 || feature->uvs.at(0)[3](0) != 21)
    throw std::runtime_error("merge changed observation order or payload precedence");
  feature->clean_invalid_measurements({});
  feature->clean_older_measurements(2); // Inclusive, even for unsorted histories.
  if (feature->timestamps.at(0) != std::vector<double>{3} ||
      feature->uvs.at(0)[0](0) != 30 || feature->uvs_norm.at(0)[0](1) != 33 ||
      feature->timestamps.at(1) != std::vector<double>{4})
    throw std::runtime_error("compaction separated observation time and pixel data");
  add(destination, std::numeric_limits<double>::quiet_NaN(), 50);
  feature->clean_older_measurements(3);
  if (feature->timestamps.at(0).size() != 1 || !std::isnan(feature->timestamps.at(0)[0]) ||
      feature->uvs.at(0)[0](0) != 50)
    throw std::runtime_error("compaction changed the upstream comparison predicate");
  auto nonfinite = std::make_shared<ov_core::FeatureDatabase>();
  add(nonfinite, std::numeric_limits<double>::quiet_NaN(), 60);
  add(nonfinite, 5, 70);
  destination->append_new_measurements(nonfinite);
  if (feature->timestamps.at(0).size() != 3 ||
      !std::isnan(feature->timestamps.at(0)[1]) || feature->timestamps.at(0)[2] != 5 ||
      feature->uvs.at(0)[1](0) != 60)
    throw std::runtime_error("merge treated NaN as an existing observation");
  std::cout << "Feature merges and compaction preserve order, duplicates and payloads\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}
