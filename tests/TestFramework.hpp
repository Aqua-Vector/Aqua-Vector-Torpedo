#ifndef TEST_FRAMEWORK_HPP_
#define TEST_FRAMEWORK_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>

/**
 * @brief GTest-like Lightweight Test Framework for Module Valuation
 */

namespace test {

struct TestResult {
    std::string suite_name;
    std::string test_name;
    bool passed;
    std::string message;
};

class TestRunner {
public:
    static TestRunner& getInstance() {
        static TestRunner instance;
        return instance;
    }

    void addResult(const std::string& suite, const std::string& name, bool passed, const std::string& msg = "") {
        results_.push_back({suite, name, passed, msg});
    }

    void report() {
        int passed_count = 0;
        std::cout << "\n==============================================================\n";
        std::cout << "                 MODULE VALUATION TEST REPORT                 \n";
        std::cout << "==============================================================\n";
        
        for (const auto& r : results_) {
            std::cout << (r.passed ? " [ PASS ] " : " [ FAIL ] ") 
                      << r.suite_name << "." << r.test_name;
            if (!r.passed) std::cout << " -> " << r.message;
            std::cout << std::endl;
            if (r.passed) passed_count++;
        }

        std::cout << "--------------------------------------------------------------\n";
        std::cout << " SUMMARY: " << passed_count << " / " << results_.size() << " tests passed.\n";
        std::cout << "==============================================================\n" << std::endl;
    }

    bool allPassed() const {
        for (const auto& r : results_) if (!r.passed) return false;
        return true;
    }

private:
    std::vector<TestResult> results_;
};

} // namespace test

#define EXPECT_TRUE(condition) \
    test::TestRunner::getInstance().addResult(TestSuiteName, TestName, (condition), #condition " is false")

#define EXPECT_FALSE(condition) \
    test::TestRunner::getInstance().addResult(TestSuiteName, TestName, !(condition), #condition " is true")

#define EXPECT_EQ(val1, val2) \
    test::TestRunner::getInstance().addResult(TestSuiteName, TestName, (val1 == val2), std::to_string(val1) + " != " + std::to_string(val2))

#define EXPECT_NEAR(val1, val2, abs_error) \
    test::TestRunner::getInstance().addResult(TestSuiteName, TestName, (std::abs((val1) - (val2)) <= abs_error), \
        std::to_string(val1) + " is not near " + std::to_string(val2) + " (err: " + std::to_string(std::abs((val1) - (val2))) + ")")

#define TEST(Suite, Name) \
    void Suite##_##Name(const char* TestSuiteName, const char* TestName)

// Manual execution macro for simple runner
#define RUN_TEST(Suite, Name) \
    { \
        Suite##_##Name(#Suite, #Name); \
    }

#endif /* TEST_FRAMEWORK_HPP_ */
