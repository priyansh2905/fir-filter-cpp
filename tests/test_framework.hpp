#ifndef TEST_FRAMEWORK_HPP
#define TEST_FRAMEWORK_HPP

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <functional>
#include <chrono>

namespace test_framework {

struct TestCase {
    std::string suiteName;
    std::string testName;
    std::function<void()> func;
};

class TestRegistry {
public:
    static TestRegistry& instance() {
        static TestRegistry reg;
        return reg;
    }

    void registerTest(const std::string& suite, const std::string& name, std::function<void()> func) {
        m_tests.push_back({suite, name, std::move(func)});
    }

    int runAll() {
        int passed = 0;
        int failed = 0;
        std::cout << "========================================================\n";
        std::cout << " Running " << m_tests.size() << " automated FIR filter unit tests...\n";
        std::cout << "========================================================\n";

        auto startTime = std::chrono::high_resolution_clock::now();

        for (const auto& t : m_tests) {
            std::cout << "[ RUN      ] " << t.suiteName << "." << t.testName << "\n";
            m_currentTestFailed = false;
            m_failureMessage.clear();
            try {
                t.func();
            } catch (const std::exception& e) {
                m_currentTestFailed = true;
                m_failureMessage = std::string("Unexpected exception thrown: ") + e.what();
            } catch (...) {
                m_currentTestFailed = true;
                m_failureMessage = "Unknown non-standard exception thrown.";
            }

            if (m_currentTestFailed) {
                ++failed;
                std::cout << "[  FAILED  ] " << t.suiteName << "." << t.testName << "\n";
                if (!m_failureMessage.empty()) {
                    std::cout << "             Error: " << m_failureMessage << "\n";
                }
            } else {
                ++passed;
                std::cout << "[       OK ] " << t.suiteName << "." << t.testName << "\n";
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        std::cout << "========================================================\n";
        std::cout << " Test Results: " << passed << " passed, " << failed << " failed in "
                  << elapsedMs << " ms\n";
        std::cout << "========================================================\n";

        return failed == 0 ? 0 : 1;
    }

    void recordFailure(const std::string& file, int line, const std::string& msg) {
        m_currentTestFailed = true;
        m_failureMessage = file + ":" + std::to_string(line) + ": " + msg;
    }

private:
    std::vector<TestCase> m_tests;
    bool m_currentTestFailed{false};
    std::string m_failureMessage;
};

struct AutoRegister {
    AutoRegister(const std::string& suite, const std::string& name, std::function<void()> func) {
        TestRegistry::instance().registerTest(suite, name, std::move(func));
    }
};

} // namespace test_framework

#define TEST(suite_name, test_name) \
    void suite_name##_##test_name##_test(); \
    static test_framework::AutoRegister suite_name##_##test_name##_reg(#suite_name, #test_name, suite_name##_##test_name##_test); \
    void suite_name##_##test_name##_test()

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, "EXPECT_TRUE(" #condition ") failed"); \
            return; \
        } \
    } while (0)

#define EXPECT_FALSE(condition) \
    do { \
        if (condition) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, "EXPECT_FALSE(" #condition ") failed"); \
            return; \
        } \
    } while (0)

#define EXPECT_EQ(val1, val2) \
    do { \
        if ((val1) != (val2)) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, \
                "EXPECT_EQ(" #val1 ", " #val2 ") failed: " + std::to_string(val1) + " != " + std::to_string(val2)); \
            return; \
        } \
    } while (0)

#define EXPECT_NEAR(val1, val2, tolerance) \
    do { \
        double diff = std::abs(static_cast<double>(val1) - static_cast<double>(val2)); \
        if (diff > static_cast<double>(tolerance)) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, \
                "EXPECT_NEAR(" #val1 ", " #val2 ", " #tolerance ") failed: |" + \
                std::to_string(val1) + " - " + std::to_string(val2) + "| = " + \
                std::to_string(diff) + " > " + std::to_string(tolerance)); \
            return; \
        } \
    } while (0)

#define EXPECT_FLOAT_EQ(val1, val2) EXPECT_NEAR(val1, val2, 1e-5f)

#define EXPECT_THROW(statement, exception_type) \
    do { \
        bool caught = false; \
        try { \
            statement; \
        } catch (const exception_type&) { \
            caught = true; \
        } catch (...) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, \
                "EXPECT_THROW(" #statement ", " #exception_type ") caught different exception"); \
            return; \
        } \
        if (!caught) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, \
                "EXPECT_THROW(" #statement ", " #exception_type ") did not throw"); \
            return; \
        } \
    } while (0)

#define EXPECT_NO_THROW(statement) \
    do { \
        try { \
            statement; \
        } catch (const std::exception& e) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, \
                std::string("EXPECT_NO_THROW(" #statement ") threw: ") + e.what()); \
            return; \
        } catch (...) { \
            test_framework::TestRegistry::instance().recordFailure(__FILE__, __LINE__, \
                "EXPECT_NO_THROW(" #statement ") threw non-std exception"); \
            return; \
        } \
    } while (0)

#endif // TEST_FRAMEWORK_HPP
