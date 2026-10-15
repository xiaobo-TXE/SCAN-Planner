// ============================================================================
// 文件：src/gradient_descent_optimizer.cpp
//
// 【说明】这是梯度下降优化器的实现，本项目当前并未调用它（用的是 L-BFGS）。
//   它的思路：每次沿“梯度反方向”走一步，步长用 Armijo 条件自动缩小，
//   直到梯度足够小或达到迭代上限。
// ============================================================================
#include <bspline_opt/gradient_descent_optimizer.h>

#define RESET "\033[0m"
#define RED "\033[31m"

GradientDescentOptimizer::RESULT
// 优化主循环：算梯度 → 定步长（回溯线搜索）→ 更新变量 → 判断收敛。
GradientDescentOptimizer::optimize(Eigen::VectorXd &x_init_optimal, double &opt_f)
{
    // 参数检查：收敛阈值必须有效，否则直接失败
    if (min_grad_ < 1e-10)
    {
        cout << RED << "min_grad_ is invalid:" << min_grad_ << RESET << endl;
        return FAILED;
    }
    // 迭代上限也必须有效
    if (iter_limit_ <= 2)
    {
        cout << RED << "iter_limit_ is invalid:" << iter_limit_ << RESET << endl;
        return FAILED;
    }

    // 取出用户数据指针（会原样传给目标函数回调）
    void *f_data = f_data_;
    // 迭代计数、调用计数（都从 2 开始，因为前面已经调用过两次）
    int iter = 2;
    int invoke_count = 2;
    // 目标函数可以通过这个标志要求“立刻提前返回”
    bool force_return;
    // 两组变量交替使用，避免每次迭代都重新分配内存
    Eigen::VectorXd x_k(x_init_optimal), x_kp1(x_init_optimal.rows());
    // 对应的代价值，以及历史最小值
    double cost_k, cost_kp1, cost_min;
    // 两组梯度向量
    Eigen::VectorXd grad_k(x_init_optimal.rows()), grad_kp1(x_init_optimal.rows());

    // 先算初始点处的代价与梯度
    cost_k = objfun_(x_k, grad_k, force_return, f_data);
    // 目标函数要求提前返回 → 直接返回
    if (force_return)
        return RETURN_BY_ORDER;
    cost_min = cost_k;
    // 取梯度里绝对值最大的分量
    double max_grad = max(abs(grad_k.maxCoeff()), abs(grad_k.minCoeff()));
    constexpr double MAX_MOVEMENT_AT_FIRST_ITERATION = 0.1; // meter
    // 第一步步长：限制第一次移动不超过 0.1 米，避免一上来就冲太远
    double alpha0 = max_grad < MAX_MOVEMENT_AT_FIRST_ITERATION ? 1.0 : (MAX_MOVEMENT_AT_FIRST_ITERATION / max_grad);
    // 沿梯度反方向走一步
    x_kp1 = x_k - alpha0 * grad_k;
    // 算新点处的代价与梯度
    cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    // 更新历史最小代价
    if (cost_min > cost_kp1)
        cost_min = cost_kp1;

    /*** start iteration ***/
    // 主迭代循环：直到达到“迭代次数”或“函数调用次数”上限
    while (++iter <= iter_limit_ && invoke_count <= invoke_limit_)
    {
        // 位移 s 和梯度变化 y
        Eigen::VectorXd s = x_kp1 - x_k;
        Eigen::VectorXd y = grad_kp1 - grad_k;
        // Barzilai-Borwein 步长：用位移与梯度变化估计一个合适的步长
        double alpha = s.dot(y) / y.dot(y);
        // 步长算出 NaN/Inf（数值异常）→ 直接失败退出
        if (isnan(alpha) || isinf(alpha))
        {
            cout << RED << "step size invalid! alpha=" << alpha << RESET << endl;
            return FAILED;
        }

        // 奇偶迭代交替使用两组变量，省掉一次拷贝
        if (iter % 2) // to avoid copying operations
        {
            // 回溯线搜索：不满足 Armijo 条件就把步长减半，再试一次
            do
            {
                // 沿梯度反方向走一步
                x_k = x_kp1 - alpha * grad_kp1;
                // 算新点的代价与梯度
                cost_k = objfun_(x_k, grad_k, force_return, f_data);
                // 函数调用次数 +1（用于限制总开销）
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                // 步长减半，继续试
                alpha *= 0.5;
            // Armijo 条件：代价必须下降得足够多，才接受这一步
            } while (cost_k > cost_kp1 - 1e-4 * alpha * grad_kp1.transpose() * grad_kp1); // Armijo condition

            // 梯度已经足够小 → 认为找到最小值
            if (grad_k.norm() < min_grad_)
            {
                // 把最优代价写回给调用者
                opt_f = cost_k;
                // 返回“找到最小值”
                return FIND_MIN;
            }
        }
        // 偶数次迭代：用另一组变量做同样的事（避免拷贝）
        else
        {
            // 同样做回溯线搜索
            do
            {
                // 沿梯度反方向走一步
                x_kp1 = x_k - alpha * grad_k;
                // 算新点的代价与梯度
                cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                alpha *= 0.5;
            // Armijo 条件（与上面完全相同）
            } while (cost_kp1 > cost_k - 1e-4 * alpha * grad_k.transpose() * grad_k); // Armijo condition

            // 梯度足够小 → 找到最小值
            if (grad_kp1.norm() < min_grad_)
            {
                // 返回最优代价
                opt_f = cost_kp1;
                return FIND_MIN;
            }
        }
    }

    opt_f = iter_limit_ % 2 ? cost_k : cost_kp1;
    return REACH_MAX_ITERATION;
}
