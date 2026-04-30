// segment_check.cpp - 정지 구간만 추출해서 spike 분석
// 빌드: g++ -O2 -std=c++17 -o segment_check segment_check.cpp
// 실행: ./segment_check imu_motion.csv

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>

struct Segment {
    const char* name;
    double t_start_us;
    double t_end_us;
};

void analyze(const std::vector<float>& v, const char* axis_name) {
    if (v.size() < 100) {
        printf("  %s: 데이터 부족 (%zu)\n", axis_name, v.size());
        return;
    }
    
    std::vector<float> diff;
    diff.reserve(v.size() - 1);
    for (size_t i = 1; i < v.size(); ++i)
        diff.push_back(std::fabs(v[i] - v[i-1]));
    
    double sum = 0, sum_sq = 0;
    for (float d : diff) { sum += d; sum_sq += (double)d * d; }
    double mean = sum / diff.size();
    double std = std::sqrt(sum_sq / diff.size() - mean * mean);
    
    int s10 = 0, s50 = 0;
    float max_d = 0;
    for (float d : diff) {
        if (d > mean + 10 * std) s10++;
        if (d > mean + 50 * std) s50++;
        if (d > max_d) max_d = d;
    }
    
    printf("  %8s: std=%.5f, max=%.4f (%5.1fσ), >10σ=%d, >50σ=%d\n",
           axis_name, std, max_d, max_d / std, s10, s50);
}

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "imu_motion.csv";
    
    FILE* fp = fopen(path, "r");
    if (!fp) { perror("fopen"); return 1; }
    
    char line[256];
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return 1; }
    
    // 정지 구간 정의 (마이크로초)
    Segment segs[] = {
        {"정지 1 (0~25초)",    0,           25e6},
        {"운동 (35~265초)",    35e6,        265e6},
        {"정지 2 (275~300초)", 275e6,       300e6},
    };
    int N_SEGS = sizeof(segs) / sizeof(segs[0]);
    
    // 각 segment별 6축 데이터
    std::vector<std::vector<float>> ax(N_SEGS), ay(N_SEGS), az(N_SEGS);
    std::vector<std::vector<float>> gx(N_SEGS), gy(N_SEGS), gz(N_SEGS);
    
    double t_us;
    float v[6];
    while (fgets(line, sizeof(line), fp)) {
        if (sscanf(line, "%lf,%f,%f,%f,%f,%f,%f",
                   &t_us, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 7) continue;
        
        for (int s = 0; s < N_SEGS; ++s) {
            if (t_us >= segs[s].t_start_us && t_us < segs[s].t_end_us) {
                ax[s].push_back(v[0]); ay[s].push_back(v[1]); az[s].push_back(v[2]);
                gx[s].push_back(v[3]); gy[s].push_back(v[4]); gz[s].push_back(v[5]);
                break;
            }
        }
    }
    fclose(fp);
    
    for (int s = 0; s < N_SEGS; ++s) {
        printf("\n========================================\n");
        printf("%s — 샘플 수: %zu\n", segs[s].name, ax[s].size());
        printf("========================================\n");
        analyze(ax[s], "ax_g");
        analyze(ay[s], "ay_g");
        analyze(az[s], "az_g");
        analyze(gx[s], "gx_dps");
        analyze(gy[s], "gy_dps");
        analyze(gz[s], "gz_dps");
    }
    
    return 0;
}