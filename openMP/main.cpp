#include <iostream>
#include <vector>
#include <omp.h>

int main() {
    const int n = 100;
    std::vector<int> array(n);

    // 初始化数组 1-100
    for (int i = 0; i < n; ++i) {
        array[i] = i + 1;
    }

    int sum = 0;

    // 使用OpenMP并行化计算数组元素和
    #pragma omp parallel for reduction(+:sum)
    for (int i = 0; i < n; ++i) {
        sum += array[i];
        // 
        std::cout << "Sum: " << sum << std::endl;
    }

    std::cout << "o Sum: " << sum << std::endl;
    return 0;
}
