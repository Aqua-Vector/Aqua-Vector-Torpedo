// spike_check.cpp - Median filter 필요성 검증
// 빌드: g++ -O2 -std=c++17 -o spike_check spike_check.cpp
// 실행: ./spike_check imu_1h.csv

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

struct AxisStats {
    const char* name;
    std::vector<float> values;
    float median_diff = 0;
    float std_diff = 0;
    float max_diff = 0;
    int spike_10sigma = 0;
    int spike_50sigma = 0;
    int spike_100sigma = 0;
};

// CSV 한 줄 파싱 — 7개 float (t_us, ax, ay, az, gx, gy, gz)
bool parse_line(const char* line, double& t_us, float v[6]) {
    return sscanf(line, "%lf,%f,%f,%f,%f,%f,%f",
                  &t_us, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 7;
}

// 정렬된 벡터의 중간값
float median_of(std::vector<float>& v) {
    if (v.empty()) return 0;
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "imu_1h.csv";
    
    FILE* fp = fopen(path, "r");
    if (!fp) { perror("fopen"); return 1; }

    printf("Loading %s...\n", path);

    AxisStats axes[6] = {
        {"ax_g",   {}}, {"ay_g",   {}}, {"az_g",   {}},
        {"gx_dps", {}}, {"gy_dps", {}}, {"gz_dps", {}}
    };
    for (auto& a : axes) a.values.reserve(3000000);

    char line[256];
    fgets(line, sizeof(line), fp);  // skip header

    long N = 0;
    double t_us;
    float v[6];
    while (fgets(line, sizeof(line), fp)) {
        if (!parse_line(line, t_us, v)) continue;
        for (int i = 0; i < 6; ++i) axes[i].values.push_back(v[i]);
        N++;
    }
    fclose(fp);
    printf("  Loaded %ld samples\n\n", N);
    
    if (N < 100) {
        printf("Not enough samples\n");
        return 1;
    }

    // 각 축 통계
    for (auto& ax : axes) {
        // 인접 샘플 차이의 절댓값
        std::vector<float> diff;
        diff.reserve(ax.values.size() - 1);
        for (size_t i = 1; i < ax.values.size(); ++i) {
            diff.push_back(std::fabs(ax.values[i] - ax.values[i-1]));
        }
        
        // median
        std::vector<float> diff_copy = diff;  // median 계산이 partial sort
        ax.median_diff = median_of(diff_copy);
        
        // std (mean과 std)
        double sum = 0, sum_sq = 0;
        for (float d : diff) { sum += d; sum_sq += (double)d * d; }
        double mean = sum / diff.size();
        ax.std_diff = std::sqrt(sum_sq / diff.size() - mean * mean);
        
        // spike count + max
        float thr10 = mean + 10.0f * ax.std_diff;
        float thr50 = mean + 50.0f * ax.std_diff;
        float thr100 = mean + 100.0f * ax.std_diff;
        for (float d : diff) {
            if (d > thr10) ax.spike_10sigma++;
            if (d > thr50) ax.spike_50sigma++;
            if (d > thr100) ax.spike_100sigma++;
            if (d > ax.max_diff) ax.max_diff = d;
        }
    }

    // 출력
    printf("======================================================================\n");
    printf("SPIKE 검출 — 인접 샘플 간 차이 분석\n");
    printf("======================================================================\n\n");

    printf("%10s | %14s | %14s | %12s | %12s | %12s\n",
           "축", "median |Δ|", "std |Δ|", ">10σ", ">50σ", ">100σ");
    printf("---------- | -------------- | -------------- | ------------ | ------------ | ------------\n");

    for (const auto& ax : axes) {
        double r10 = 100.0 * ax.spike_10sigma / (ax.values.size() - 1);
        double r50 = 100.0 * ax.spike_50sigma / (ax.values.size() - 1);
        double r100 = 100.0 * ax.spike_100sigma / (ax.values.size() - 1);
        printf("%10s | %14.6f | %14.6f | %5d (%5.3f%%) | %5d (%5.3f%%) | %5d (%5.3f%%)\n",
               ax.name, ax.median_diff, ax.std_diff,
               ax.spike_10sigma, r10,
               ax.spike_50sigma, r50,
               ax.spike_100sigma, r100);
    }

    printf("\n");
    printf("======================================================================\n");
    printf("최대값 분석 — 가장 큰 단발 변화\n");
    printf("======================================================================\n\n");

    printf("%10s | %14s | %14s | %14s\n",
           "축", "max |Δ|", "max / median", "max / std");
    printf("---------- | -------------- | -------------- | --------------\n");

    for (const auto& ax : axes) {
        double r_med = (ax.median_diff > 0) ? ax.max_diff / ax.median_diff : 0;
        double r_std = (ax.std_diff > 0) ? ax.max_diff / ax.std_diff : 0;
        printf("%10s | %14.6f | %12.1fx | %12.1fσ\n",
               ax.name, ax.max_diff, r_med, r_std);
    }

    // 판정
    double avg_r50_acc = 0, avg_r50_gyr = 0;
    long N_diff = N - 1;
    for (int i = 0; i < 3; ++i) {
        avg_r50_acc += 100.0 * axes[i].spike_50sigma / N_diff;
        avg_r50_gyr += 100.0 * axes[i+3].spike_50sigma / N_diff;
    }
    avg_r50_acc /= 3;
    avg_r50_gyr /= 3;

    printf("\n");
    printf("======================================================================\n");
    printf("판정\n");
    printf("======================================================================\n");
    printf("\n가속도 평균 spike (>50σ): %.4f%%\n", avg_r50_acc);
    printf("자이로 평균 spike (>50σ): %.4f%%\n", avg_r50_gyr);

    if (avg_r50_acc < 0.001 && avg_r50_gyr < 0.001) {
        printf("\n→ Median filter 불필요 (spike < 0.001%%)\n");
        printf("  결정: ADR-007에 'Median 미사용 근거'로 박음\n");
    } else if (avg_r50_acc < 0.01 && avg_r50_gyr < 0.01) {
        printf("\n→ Median filter 선택사항 (spike < 0.01%%)\n");
        printf("  결정: 일단 빼고 이상 발견 시 추가\n");
    } else {
        printf("\n→ Median filter 권장 (spike >= 0.01%%)\n");
        printf("  결정: ADR-008로 Median filter 별도 도입\n");
    }

    return 0;
}