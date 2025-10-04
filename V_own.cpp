#include <bits/stdc++.h>
#include <x86intrin.h>
#include <sched.h>
#include <unistd.h>
#include <sys/mman.h>
#include <ranges>

typedef long long ll;

using namespace std;

constexpr int MAX_MEMORY = 256 * 1024 * 1024;
constexpr int MAX_ASSOC = 1 << 5;
constexpr int READS_COUNT = 1 << 20;
constexpr int MAX_MEMORY_RANGE = 1 << 11;

uintptr_t *memory = static_cast<uintptr_t *>(
    mmap(
        nullptr,                    // 1. Let the kernel choose the starting address
        MAX_MEMORY,                 // 2. Size of the memory block in bytes
        PROT_READ | PROT_WRITE,     // 3. Memory protection: readable & writable
        MAP_PRIVATE | MAP_ANON,     // 4. Private anonymous mapping
        -1,                         // 5. File descriptor (ignored for MAP_ANON)
        0                           // 6. Offset (ignored for MAP_ANON)
    )
);

static inline uint64_t rdtsc() {
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

uintptr_t* go_memory(uintptr_t* p) {
    for (int i = 0; i < READS_COUNT; ++i) {
        p = reinterpret_cast<uintptr_t *>(*p);
    }
    return p;
}


volatile uintptr_t* another_sink;
void use_volatile(volatile uintptr_t* sink) {
    another_sink = reinterpret_cast<uintptr_t *>(*sink);
}

ll my_get_time(const int stride, const int spots) {

    std::vector<int> indices(spots);  // size = n
    std::iota(indices.begin(), indices.end(), 0); // fills 0..n-1

    std::random_device rd;   // Seed
    std::mt19937 g(rd());    // Mersenne Twister RNG
    std::ranges::shuffle(indices, g);

    const auto stride_ptr = stride / sizeof(uintptr_t);
    auto ptr = memory + indices[0] * stride_ptr;
    for (int i = 0; i < spots; i++) {
        auto next = memory + indices[(i + 1) % spots] * stride_ptr;
        *ptr = reinterpret_cast<uintptr_t>(next);
        ptr = next;
    }

    volatile uintptr_t* sink;

    sink = go_memory(memory);
    use_volatile(sink);

    auto t1 = rdtsc();
    sink = go_memory(memory);
    auto t2 = rdtsc();

    use_volatile(sink);

    // std::cout << t2 - t1 << "\n";

    return t2 - t1;
}

bool is_movement(
    const vector <int> &jumps_old,
    const vector <int> &jumps_cur
) {
    return jumps_old.empty() || jumps_cur.empty() || jumps_old.front() != jumps_cur.front();
}

void detect_entities(
    const vector <vector <int> > &jumps_by_strides,
    const vector <int> &strides,
    map<int, ll> &entities
) {
    auto last_jumps = jumps_by_strides.back();

    for (auto jump : last_jumps) {
        entities[jump] = -1;
    }

    int idx = jumps_by_strides.size() - 2;
    while (idx >= 0) {
        for (auto cur_jump : jumps_by_strides[idx]) {
            if (entities.contains(cur_jump) && entities[cur_jump] == -1) {
                entities[cur_jump] = strides[idx];
            }
        }
        idx--;
    }


    for (auto jump : last_jumps) {
        if (entities[jump] == -1) {
            entities.erase(jump);
        }
    }
}

void detect_phase(map<int, ll> &entities) {
    int stride = 1 << 3;

    vector <vector <int> > jumps_by_strides;
    vector <int> strides;

    while (stride * MAX_ASSOC < MAX_MEMORY) {
        int spots = 1;
        ll old_time = -1;

        vector <int> jumps;

        while (spots < MAX_ASSOC) {
            ll cur_time = my_get_time(stride, spots);

            if (old_time != -1 && cur_time > old_time * 1.25) {
                jumps.push_back(spots - 1);
            }
            old_time = cur_time;
            spots++;
        }

        std::cerr << "Stride: " << stride << " Jumps: " << jumps.size() << endl;
        for (auto jump : jumps) {
            std::cerr << jump << " ";
        }
        std::cerr << endl;

        bool should_break = !jumps_by_strides.empty() && !is_movement(jumps_by_strides.back(), jumps);
        jumps_by_strides.push_back(jumps);
        strides.push_back(stride);

        if (should_break) {
            break;
        }

        stride *= 2;
    }

    detect_entities(jumps_by_strides, strides, entities);
}

ll get_time_high_low_stride(const int high_stride, const int low_stride, const int max_spots) {
    const int stride = high_stride + low_stride;
    int spots = 1;
    ll total = 0;
    while (spots <= max_spots) {
        total += my_get_time(stride, spots);
        spots++;
    }
    return total / max_spots;
}

enum PATTERN {
    PATTERN_UP,
    PATTERN_DOWN,
    PATTERN_SAME
};

const double jumpCoef = 1.15;

int disambiguation_phase() {

    vector <pair<int, PATTERN>> patterns = {};

    for (int high_stride = 16; high_stride <= 1024; high_stride *= 2) {

        auto time_zero_low = get_time_high_low_stride(high_stride, 0, MAX_MEMORY_RANGE);

        vector count_changes = {0, 0};
        for (int low_stride = max(1, high_stride >> 4); low_stride < high_stride; low_stride *= 2) {

            auto time_cur_low = get_time_high_low_stride(high_stride, low_stride, MAX_MEMORY_RANGE);

            if (time_cur_low <= jumpCoef * time_zero_low && time_zero_low <= jumpCoef * time_cur_low) {
                continue;
            }

            if (time_cur_low < time_zero_low) {
                count_changes[PATTERN_UP]++;
            } else {
                count_changes[PATTERN_DOWN]++;
            }
        }

        auto pat = PATTERN_SAME;
        if (count_changes[PATTERN_UP] > count_changes[PATTERN_DOWN]) pat = PATTERN_UP;
        else if (count_changes[PATTERN_UP] < count_changes[PATTERN_DOWN]) pat = PATTERN_DOWN;

        patterns.emplace_back(high_stride, pat);
        std::cerr << "Stride: " << high_stride << " Pattern: " << pat << std::endl;
        std::cerr << count_changes[PATTERN_UP] << " " << count_changes[PATTERN_DOWN] << std::endl;
    }

    int line_size = -1;
    bool observed_down = false;
    for (auto [high_stride, pat] : patterns) {
        if ((pat == PATTERN_UP || pat == PATTERN_SAME) && observed_down) {
            line_size = high_stride;
            break;
        }
        if (pat == PATTERN_DOWN) {
            observed_down = true;
        }
    }
    return line_size;
}

void set_cpu() {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(0, &set);
    if (sched_setaffinity(0, sizeof(set), &set) == -1) {
        perror("sched_setaffinity");
        exit(1);
    }
}

int main(int argc, char **argv) {

    // ios::sync_with_stdio(false);
    // cin.tie(nullptr);

    set_cpu();

    map<int, ll> entities;
    detect_phase(entities);

    std::cout << "Entities: " << entities.size() << endl;

    if (entities.empty()) {
        std::cout << "No entities found" << endl;
        return 0;
    }

    for (auto [key, value] : entities) {
        std::cout << "Entity associativity: " << key << ", entity stride: " << value << endl;
    }
    std::cout << "Cache size: " << entities.begin()->first * entities.begin()->second << "B" << endl;
    std::cout << "Cache associativity: " << entities.begin()->first << endl;


    auto line_size = disambiguation_phase();
    std::cout << "Cache line size: " << line_size << "B" << endl;
    return 0;
}
