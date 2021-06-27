// Angle value at the initial moment (in radians)
// The actual initial position of manipulator should be: [0-90 0 0 0]
double t1_current = 0;
double t2_current = 0;
double t3_current = 90*PI/180;
double t4_current = 0;
double t5_current = 0;
double t6_current = 0;

//TODO:Inverse kinematics of manipulator
MatrixXd robot_ik(MatrixXd& T06)
{
	//Mechanical arm bar parameters
	const double a1 = 0.0231;
	const double a2 = 0.26;
	const double a3 = 0.0276;
	const double d1 = 0.3525;
	const double d4 = 0.2764;
	const double d6 = 0.1319;
	
	//! To calculate t1
	//double t1_min = -170 * PI / 180;
	//double t1_max = 170 * PI / 180;
	double px = T06(0, 3);
	double py = T06(1, 3);
	double pz = T06(2, 3);
	MatrixXd p(3, 1);
	p << px, py, pz;
	double nx = T06(0, 2);
	double ny = T06(1, 2);
	double nz = T06(2, 2);
	MatrixXd n(3, 1);
	n << nx, ny, nz;
	MatrixXd p46 = d6 * n;
	MatrixXd p04 = p - p46;
	double t1 = atan2(p04(1, 0), p04(0, 0));

	MatrixXd T01(4, 4);
	T01.row(0) << cos(t1), 0, sin(t1), a1*cos(t1);
	T01.row(1) << sin(t1), 0, -cos(t1), a1*sin(t1);
	T01.row(2) << 0, 1, 0, d1;
	T01.row(3) << 0, 0, 0, 1;


	//! To calculate t3
	double t3_min = -80 * PI / 180;
	double t3_max = 180 * PI / 180;
	vector<double> t3;//Stores possible T3 values
	MatrixXd p01(3, 1);
	p01 << a1 * cos(t1), a1*sin(t1), d1;
	MatrixXd p14 = p04 - p01;
	double l4 = sqrt(d4*d4 + a3 * a3);
	if (fabs((pow(l4, 2) + pow(a2, 2) - pow(p14.norm(), 2)) / (2 * a2*l4)) <= 1)//because acos(x),x cannot >1 or <-1, or you get an imaginary number
	{
		double phi = acos((pow(l4, 2) + pow(a2, 2) - pow(p14.norm(), 2)) / (2 * a2*l4));
		//cout << "phi= " << phi << endl;
		double w = atan2(d4, a3);
		double t3_1 = (phi + w) - PI;
		double t3_2 = -t3_1;
		/*double t3_3 = PI - (phi - w);
		double t3_4 = -t3_3;*/
		t3.push_back(t3_1);
		t3.push_back(t3_2);
		/*t3.push_back(t3_3);
		t3.push_back(t3_4);*/
	}
	auto it_t3 = t3.begin();
	while (it_t3 != t3.end())
	{
		if (*it_t3<t3_min || *it_t3>t3_max) it_t3 = t3.erase(it_t3);//Clear values that are out of the Angle range
		else ++it_t3;
	}
	//cout << "打印清除超出角度范围之后的t3" << endl;
	//for (auto x : t3) cout << x << endl;
	vector<MatrixXd> T23_vector;//Container to store T23
	for (auto i = 0; i < t3.size(); ++i)
	{
		MatrixXd T23(4, 4);
		T23.row(0) << cos(t3[i]), 0, sin(t3[i]), a3*cos(t3[i]);
		T23.row(1) << sin(t3[i]), 0, -cos(t3[i]), a3*sin(t3[i]);
		T23.row(2) << 0, 1, 0, 0;
		T23.row(3) << 0, 0, 0, 1;
		T23_vector.push_back(T23);
	}

	//! Calculation of t2
	double t2_min = -20 * PI / 180;
	double t2_max = 200 * PI / 180;
	vector<double> t2;//Store possible t2 values
	MatrixXd R01(3, 3);
	R01.row(0) << cos(t1), 0, sin(t1);
	R01.row(1) << sin(t1), 0, -cos(t1);
	R01.row(2) << 0, 1, 0;
	MatrixXd p14_1 = R01.transpose()*p14;
	double beta1 = atan2(p14_1(1, 0), p14_1(0, 0));
	if (fabs((a2*a2 + pow(p14.norm(), 2) - l4 * l4) / (2 * a2*p14.norm())) <= 1)
	{
		double beta2 = acos((a2*a2 + pow(p14.norm(), 2) - l4 * l4) / (2 * a2*p14.norm()));
		double t2_1 = beta1 + beta2;
		double t2_2 = -t2_1;
		double t2_3 = beta1 - beta2;
		double t2_4 = -t2_3;
		t2.push_back(t2_1);
		//t2.push_back(t2_2);
		t2.push_back(t2_3);
		//t2.push_back(t2_4);
	}

	auto it_t2 = t2.begin();
	while (it_t2 != t2.end())
	{
		if (*it_t2<t2_min || *it_t2>t2_max) it_t2 = t2.erase(it_t2);
		else ++it_t2;
	}
	//cout << "打印清除超出角度范围之后的t2" << endl;
	//for (auto x : t2) cout << x << endl;
	vector<MatrixXd> T12_vector;//存储T12的容器
	for (auto i = 0; i < t2.size(); ++i)
	{
		MatrixXd T12(4, 4);
		T12.row(0) << cos(t2[i]), -sin(t2[i]), 0, a2*cos(t2[i]);
		T12.row(1) << sin(t2[i]), cos(t2[i]), 0, a2*sin(t2[i]);
		T12.row(2) << 0, 0, 1, 0;
		T12.row(3) << 0, 0, 0, 1;
		T12_vector.push_back(T12);
	}

	//! Calculate t4 and t6
	double t4_min = -180 * PI / 180;
	double t4_max = 180 * PI / 180;
	double t6_min = -360 * PI / 180;
	double t6_max = 360 * PI / 180;
	vector<double> t4; //Stores possible t4 values
	vector<double> t6; //Stores possible t6 values
	MatrixXd T03(4, 4);
	MatrixXd R03(3, 3);
	MatrixXd R06(3, 3);
	MatrixXd R36(3, 3);
	R06 = T06.block<3, 3>(0, 0);
	/*for (int i = 0; i < T12_vector.size(); ++i)
	{
		for (int j = 0; j < T23_vector.size(); ++j)
		{
			T03 = T01 * T12_vector[i] * T23_vector[j];
			R03 = T03.block<3, 3>(0, 0);
			R36 = R03.transpose()*R06;
			double t4_1 = atan2(R36(1, 2), R36(0, 2)) + 2 * PI;
			double t4_2 = atan2(R36(1, 2), R36(0, 2)) + PI;
			double t4_3 = atan2(R36(1, 2), R36(0, 2));
			double t4_4 = atan2(R36(1, 2), R36(0, 2)) - PI;
			double t4_5 = atan2(R36(1, 2), R36(0, 2)) - 2 * PI;
			t4.push_back(t4_1);
			t4.push_back(t4_2);
			t4.push_back(t4_3);
			t4.push_back(t4_4);
			t4.push_back(t4_5);
			double t6_1 = atan2(-R36(2, 1), R36(2, 0));
			double t6_2 = t6_1 + PI;
			double t6_3 = t6_1 + 2 * PI;
			double t6_4 = t6_1 - PI;
			double t6_5 = t6_1 - 2 * PI;
			t6.push_back(t6_1);
			t6.push_back(t6_2);
			t6.push_back(t6_3);
			t6.push_back(t6_4);
			t6.push_back(t6_5);
		}
	}*/

	for (int i = 0; i < T12_vector.size() / 2.0; ++i)
	{
		for (int j = 0; j < T23_vector.size() / 2.0; ++j)
		{
			T03 = T01 * T12_vector[i] * T23_vector[j];
			R03 = T03.block<3, 3>(0, 0);
			R36 = R03.transpose()*R06;
			double t4_1 = atan2(R36(1, 2), R36(0, 2)) + 2 * PI;
			double t4_2 = atan2(R36(1, 2), R36(0, 2)) + PI;
			double t4_3 = atan2(R36(1, 2), R36(0, 2));
			double t4_4 = atan2(R36(1, 2), R36(0, 2)) - PI;
			double t4_5 = atan2(R36(1, 2), R36(0, 2)) - 2 * PI;
			//t4.push_back(t4_1);
			t4.push_back(t4_2);
			t4.push_back(t4_3);
			t4.push_back(t4_4);
			//t4.push_back(t4_5);
			double t6_1 = atan2(-R36(2, 1), R36(2, 0));
			double t6_2 = t6_1 + PI;
			double t6_3 = t6_1 + 2 * PI;
			double t6_4 = t6_1 - PI;
			double t6_5 = t6_1 - 2 * PI;
			t6.push_back(t6_1);
			t6.push_back(t6_2);
			t6.push_back(t6_3);
			t6.push_back(t6_4);
			t6.push_back(t6_5);
		}
	}

	for (int i = T12_vector.size() / 2; i < T12_vector.size(); ++i)
	{
		for (int j = T23_vector.size() / 2; j < T23_vector.size(); ++j)
		{
			T03 = T01 * T12_vector[i] * T23_vector[j];
			R03 = T03.block<3, 3>(0, 0);
			R36 = R03.transpose()*R06;
			double t4_1 = atan2(R36(1, 2), R36(0, 2)) + 2 * PI;
			double t4_2 = atan2(R36(1, 2), R36(0, 2)) + PI;
			double t4_3 = atan2(R36(1, 2), R36(0, 2));
			double t4_4 = atan2(R36(1, 2), R36(0, 2)) - PI;
			double t4_5 = atan2(R36(1, 2), R36(0, 2)) - 2 * PI;
			//t4.push_back(t4_1);
			t4.push_back(t4_2);
			t4.push_back(t4_3);
			t4.push_back(t4_4);
			//t4.push_back(t4_5);
			double t6_1 = atan2(-R36(2, 1), R36(2, 0));
			double t6_2 = t6_1 + PI;
			double t6_3 = t6_1 + 2 * PI;
			double t6_4 = t6_1 - PI;
			double t6_5 = t6_1 - 2 * PI;
			t6.push_back(t6_1);
			t6.push_back(t6_2);
			t6.push_back(t6_3);
			t6.push_back(t6_4);
			t6.push_back(t6_5);
		}
	}
	//cout << "打印t4的数量: " << t4.size() << endl;
	//cout << "打印t6的数量: " << t6.size() << endl;
	auto it_t4 = t4.begin();
	while (it_t4 != t4.end())
	{
		if (*it_t4<t4_min || *it_t4>t4_max) it_t4 = t4.erase(it_t4);//Clear values that are out of the Angle range
		else ++it_t4;
	}
	//cout << "打印清除超出角度范围之后的t4" << endl;
	//for (auto x : t4) cout << x << endl;
	vector<MatrixXd> T34_vector;//Container to store T34
	for (auto i = 0; i < t4.size(); ++i)
	{
		MatrixXd T34(4, 4);
		T34.row(0) << cos(t4[i]), 0, -sin(t4[i]), 0;
		T34.row(1) << sin(t4[i]), 0, cos(t4[i]), 0;
		T34.row(2) << 0, -1, 0, d4;
		T34.row(3) << 0, 0, 0, 1;
		T34_vector.push_back(T34);
	}
	/*cout << "打印全部的t6:" << endl;
	for (auto x : t6) cout << x << ' ';
	cout << endl;
	cout << "打印T34" << endl;
	for (auto x : T34_vector) cout << x << endl;*/
	auto it_t6 = t6.begin();
	while (it_t6 != t6.end())
	{
		if (*it_t6<t6_min || *it_t6>t6_max) it_t6 = t6.erase(it_t6);//Clear values that are out of the Angle range
		else ++it_t6;
	}
	//cout << "打印清除超出角度范围之后的t6" << endl;
	//for (auto x : t6) cout << x << endl;
	vector<MatrixXd> T56_vector;//The container to store the T56
	for (auto i = 0; i < t6.size(); ++i)
	{
		MatrixXd T56(4, 4);
		T56.row(0) << cos(t6[i]), -sin(t6[i]), 0, 0;
		T56.row(1) << sin(t6[i]), cos(t6[i]), 0, 0;
		T56.row(2) << 0, 0, 1, d6;
		T56.row(3) << 0, 0, 0, 1;
		T56_vector.push_back(T56);
	}
	//cout << "打印T56" << endl;
	//for (auto x : T56_vector) cout << x << endl;

	//! 计算t5
	double t5_min = -125 * PI / 180;
	double t5_max = 125 * PI / 180;
	vector<double> t5; //Stores possible t5 values
	double t5_1;
	double t5_2;
	double r13 = T06(0, 2); double r23 = T06(1, 2); double r33 = T06(2, 2);
	//for (int j = 0; j < t2.size(); ++j)
	//{
	//	for (int k = 0; k < t3.size(); ++k)
	//	{
	//		if ((1 - pow((cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33), 2)) >= 0)//sqrt(x),x>=0
	//		{
	//			double t5_1 = atan2(sqrt(1 - pow((cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33), 2)), cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33);
	//			double t5_2 = -t5_1;
	//			t5.push_back(t5_1);
	//			t5.push_back(t5_2);
	//		}

	//	}
	//}

	for (int j = 0; j < t2.size() / 2.0; ++j)
	{
		for (int k = 0; k < t3.size() / 2.0; ++k)
		{
			if ((1 - pow((cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33), 2)) >= 0)//sqrt(x),x>=0
			{
				t5_1 = atan2(sqrt(1 - pow((cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33), 2)), cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33);
				t5_2 = -t5_1;
				t5.push_back(t5_1);
				t5.push_back(t5_2);
			}
		}
	}
	for (int j = t2.size() / 2; j < t2.size(); ++j)
	{
		for (int k = t3.size() / 2; k < t3.size(); ++k)
		{
			if ((1 - pow((cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33), 2)) >= 0)//sqrt(x),x>=0
			{
				t5_1 = atan2(sqrt(1 - pow((cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33), 2)), cos(t1)*sin(t2[j] + t3[k])*r13 + sin(t1)*sin(t2[j] + t3[k])*r23 - cos(t2[j] + t3[k])*r33);
				t5_2 = -t5_1;
				t5.push_back(t5_1);
				t5.push_back(t5_2);
			}
		}
	}
	auto it_t5 = t5.begin();
	while (it_t5 != t5.end())
	{
		if (*it_t5<t5_min || *it_t5>t5_max) it_t5 = t5.erase(it_t5);//Clear values that are out of the Angle range
		else ++it_t5;
	}
	//cout << "打印清除超出角度范围之后的t5" << endl;
	//for (auto x : t5) cout << x << endl;
	vector<MatrixXd> T45_vector;//The container for storing T45
	for (auto i = 0; i < t5.size(); ++i)
	{
		MatrixXd T45(4, 4);
		T45.row(0) << cos(t5[i]), 0, sin(t5[i]), 0;
		T45.row(1) << sin(t5[i]), 0, -cos(t5[i]), 0;
		T45.row(2) << 0, 1, 0, 0;
		T45.row(3) << 0, 0, 0, 1;
		T45_vector.push_back(T45);
	}
	//cout << "打印T45" << endl;
	//for (auto x : T45_vector) cout << x << endl;


	//! Find the 6 angles that satisfy the condition
	MatrixXd angles(6, 1);//Stores 6 joint angles
	vector<MatrixXd> angles_vector;
	MatrixXd T(4, 4);
	const double tor = 0.001;//Two floating-point numbers are considered equal when the difference between them is less than 0.001
	double t00;
	double t11;
	double t22;
	double t03;
	double t13;
	double t23;
	/*std::cout << ' ' << T12_vector.size() << ' ' << T23_vector.size() << ' ' << T34_vector.size()
		<< ' ' << T45_vector.size() << ' ' << T56_vector.size() << endl;*/
	//for (int j = 0; j < T12_vector.size(); ++j)
	//{
	//	for (int k = 0; k < T23_vector.size(); ++k)
	//	{
	//		for (int l = 0; l < T34_vector.size(); ++l)
	//		{
	//			for (int m = 0; m < T45_vector.size(); ++m)
	//			{
	//				for (int n = 0; n < T56_vector.size(); ++n)
	//				{
	//					MatrixXd T(4, 4);
	//					T = T01 * T12_vector[j] * T23_vector[k] * T34_vector[l] * T45_vector[m] * T56_vector[n];
	//					double t00 = fabs(T(0, 0) - T06(0, 0));
	//					double t01 = fabs(T(0, 1) - T06(0, 1));
	//					double t02 = fabs(T(0, 2) - T06(0, 2));
	//					double t03 = fabs(T(0, 3) - T06(0, 3));
	//					double t10 = fabs(T(1, 0) - T06(1, 0));
	//					double t11 = fabs(T(1, 1) - T06(1, 1));
	//					double t12 = fabs(T(1, 2) - T06(1, 2));
	//					double t13 = fabs(T(1, 3) - T06(1, 3));
	//					double t20 = fabs(T(2, 0) - T06(2, 0));
	//					double t21 = fabs(T(2, 1) - T06(2, 1));
	//					double t22 = fabs(T(2, 2) - T06(2, 2));
	//					double t23 = fabs(T(2, 3) - T06(2, 3));
	//					double tor = 0.001;
	//					//! 浮点数的比较不能通过“==”的方式
	//					if (t00<tor&&t01 < tor&&t02 < tor&&t03 < tor&&t10 < tor&&t11 < tor&&t12 < tor&&t13 < tor&&t20 < tor&&t21 < tor&&t22 < tor&&t23 < tor)
	//					{
	//						/*cout << j <<' '<< k << ' ' << l << ' ' << m << ' ' << n<<' ';
	//						cout << "66666" << endl;*/
	//						angles << t1, t2[j], t3[k], t4[l], t5[m], t6[n];
	//						angles_vector.push_back(angles);
	//					}
	//				}
	//			}
	//		}
	//	}
	//}

	for (int j = 0; j < T12_vector.size() / 2.0; ++j)
	{
		for (int k = 0; k < T23_vector.size() / 2.0; ++k)
		{
			for (int l = 0; l <= T34_vector.size() / 2; ++l)
			{
				for (int m = 0; m < T45_vector.size() / 2; ++m)
				{
					for (int n = 0; n < T56_vector.size() / 2; ++n)
					{
						T = T01 * T12_vector[j] * T23_vector[k] * T34_vector[l] * T45_vector[m] * T56_vector[n];

						t00 = fabs(T(0, 0) - T06(0, 0));
						t11 = fabs(T(1, 1) - T06(1, 1));
						t22 = fabs(T(2, 2) - T06(2, 2));
						t03 = fabs(T(0, 3) - T06(0, 3));
						t13 = fabs(T(1, 3) - T06(1, 3));
						t23 = fabs(T(2, 3) - T06(2, 3));

						//! Floating-point numbers cannot be compared using the == method
						if (t03 < tor&&t13 < tor&&t23 < tor&&t00 < tor&&t11 < tor&&t22 < tor)
						{
							angles << t1, t2[j], t3[k], t4[l], t5[m], t6[n];
							angles_vector.push_back(angles);
						}
					}
				}
			}
		}
	}
	//cout << "当前angles_vector里有多少组解：" << angles_vector.size() << endl;
	for (int j = T12_vector.size() / 2; j < T12_vector.size(); ++j)
	{
		for (int k = T23_vector.size() / 2; k < T23_vector.size(); ++k)
		{
			for (int l = T34_vector.size() / 2; l < T34_vector.size(); ++l)
			{
				for (int m = T45_vector.size() / 2; m < T45_vector.size(); ++m)
				{
					for (int n = T56_vector.size() / 2; n < T56_vector.size(); ++n)
					{
						T = T01 * T12_vector[j] * T23_vector[k] * T34_vector[l] * T45_vector[m] * T56_vector[n];
						t00 = fabs(T(0, 0) - T06(0, 0));
						t11 = fabs(T(1, 1) - T06(1, 1));
						t22 = fabs(T(2, 2) - T06(2, 2));
						t03 = fabs(T(0, 3) - T06(0, 3));
						t13 = fabs(T(1, 3) - T06(1, 3));
						t23 = fabs(T(2, 3) - T06(2, 3));
						//! Floating-point numbers cannot be compared using the == method
						if (t03 < tor&&t13 < tor&&t23 < tor&&t00 < tor&&t11 < tor&&t22 < tor)
						{
							/*cout << j <<' '<< k << ' ' << l << ' ' << m << ' ' << n<<' ';
							cout << "66666" << endl;*/
							angles << t1, t2[j], t3[k], t4[l], t5[m], t6[n];
							angles_vector.push_back(angles);
						}
					}
				}
			}
		}
	}

	cout << "There are several groups of angles:" << angles_vector.size() << endl;
	for (auto x : angles_vector) cout << x * 180 / PI << endl << endl;

	//! Find the Angle of final return
	MatrixXd ret(6, 1);//The result of the final return
	int joint1_coeff = 10;//Joint coefficient, rule: let the facet joint move as much as possible
	int joint2_coeff = 8;
	int joint3_coeff = 6;
	int joint4_coeff = 4;
	int joint5_coeff = 2;
	int joint6_coeff = 1;

	double error = 999999.0;
	double t1_diff;
	double t2_diff;
	double t3_diff;
	double t4_diff;
	double t5_diff;
	double t6_diff;

	if(angles_vector.size()!=0)
	{
		for (int i = 0; i < angles_vector.size(); ++i)
		{

			t1_diff = fabs((angles_vector[i](0, 0) - t1_current)*joint1_coeff);
			t2_diff = fabs((angles_vector[i](1, 0) - t2_current)*joint2_coeff);
			t3_diff = fabs((angles_vector[i](2, 0) - t3_current)*joint3_coeff);
			t4_diff = fabs((angles_vector[i](3, 0) - t4_current)*joint4_coeff);
			t5_diff = fabs((angles_vector[i](4, 0) - t5_current)*joint5_coeff);
			t6_diff = fabs((angles_vector[i](5, 0) - t6_current)*joint6_coeff);
			double diff_sum = t1_diff + t2_diff + t3_diff + t4_diff + t5_diff + t6_diff;
			if (diff_sum < error)//Keep updating the ones with the least error
			{
				error = diff_sum;
				ret << angles_vector[i](0, 0), angles_vector[i](1, 0), angles_vector[i](2, 0), angles_vector[i](3, 0),
					angles_vector[i](4, 0), angles_vector[i](5, 0);
			}
		}
		//The set of solutions with the least error. Updates the current Angle
		t1_current = ret(0, 0);
		t2_current = ret(1, 0);
		t3_current = ret(2, 0);
		t4_current = ret(3, 0);
		t5_current = ret(4, 0);
		t6_current = ret(5, 0);
		return ret;
	}else
	{
		ret << t1_current, t2_current, t3_current, t4_current, t5_current, t6_current;
		return ret;
	}
	
}
