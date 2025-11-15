#pragma once
class Kalman
{
public:
	float X_last; //上一时刻的最优结果  X(k-|k-1)
	float X_mid;  //当前时刻的预测结果  X(k|k-1)
	float X_now;  //当前时刻的最优结果  X(k|k)
	float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
	float P_now;  //当前时刻最优结果的协方差  P(k|k)
	float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
	float kg;     //kalman增益
	float A;      //系统参数
	float B;
	float Q;
	float R;
	float H;
	/**
	  * @name   kalmanCreate
	  * @brief  创建一个卡尔曼滤波器
	  * @param  p:  滤波器
	  *         T_Q:系统噪声协方差
	  *         T_R:测量噪声协方差
	  *
	  * @retval none
	  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
	  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
	  */
	Kalman(float T_Q, float T_R)
	{
		X_last = (float)0;
		P_last = 0;
		Q = T_Q;
		R = T_R;
		A = 1;
		B = 0;
		H = 1;
		X_mid = X_last;
	}

	float Filter(float dat)
	{
		/* -------------------- 1. 预测 (Predict) -------------------- */
		// X_mid = A * X_last;  (因为 A=1)
		// 步骤一：预测当前时刻的值。
		// 你的模型是 "当前值 = 上一刻的最优值"。
		X_mid = A * X_last;                   // 对应公式(1): X(k|k-1) = A*X(k-1|k-1)

		// P_mid = A * P_last * A' + Q; (因为 A=1, A'=1)
		// 步骤二：预测当前时刻的协方差（不确定性）。
		// 你的不确定性 = 上一刻的不确定性(P_last) + 模型本身带来的不确定性(Q)。
		P_mid = A * P_last + Q;               // 对应公式(2): P(k|k-1) = A*P(k-1|k-1)*A' + Q

		/* -------------------- 2. 更新 (Update) -------------------- */
		// 步骤三：计算卡尔曼增益 (kg)。
		// 这是最关键的一步！它是一个 0~1 之间的值，用来平衡“预测”和“测量”。
		// kg = 预测协方差 / (预测协方差 + 测量协方差)
		kg = P_mid / (P_mid + R);           // 对应公式(4): kg = P(k|k-1)*H' / (H*P(k|k-1)*H' + R)

		// 步骤四：更新（修正）当前时刻的最优估计值 (X_now)。
		// 最优值 = 预测值 + 增益 * (测量值 - 预测值)
		// (dat - X_mid) 是“残差”，即测量值和预测值之间的差距。
		X_now = X_mid + kg * (dat - X_mid);   // 对应公式(3): X(k|k) = X(k|k-1) + kg*(Z(k) - H*X(k|k-1))

		// 步骤五：更新当前时刻的协方差（不确定性）。
		// 因为我们引入了测量值，所以我们的不确定性降低了。
		// (1-kg) 总是小于 1，所以 P_now 总是小于 P_mid。
		P_now = (1 - kg) * P_mid;             // 对应公式(5): P(k|k) = (I - kg*H) * P(k|k-1)

		/* ----------------- 3. 为下一次迭代准备 ----------------- */
		P_last = P_now;                     // 将当前的最优协方差 作为 下一轮的“上一刻协方差”
		X_last = X_now;                     // 将当前的最优估计值 作为 下一轮的“上一刻估计值”

		return X_now;						// 返回当前计算出的最优估计值
	}
};