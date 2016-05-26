#ifndef WL_POINT_TYPES_H_
#define WL_POINT_TYPES_H_

namespace wl
{

	struct LASPoint
	{
		float x;
		float y;
		float z;

		float intensity;
		unsigned char return_number : 3;
		unsigned char number_of_returns_of_given_pulse : 3;
		unsigned char classification;
		char scan_angle_rank;
		unsigned char rgb[3];

		unsigned char user_data;

		LASPoint()
		{
			x = 0;
			y = 0;
			z = 0;
			intensity = 0;

			number_of_returns_of_given_pulse = 0;
			return_number = 0;
			classification = 0;
			scan_angle_rank = 0;
			user_data = 0;
		};
	};

	struct PointXYZ
	{
		inline PointXYZ (const PointXYZ &p)
		{
			x = p.x; y = p.y; z = p.z;
		}

		inline PointXYZ ()
		{
			x = y = z = 0.0f;
		}

		float x;
		float y;
		float z;
	};

	struct PointXYZI
	{
		inline PointXYZI (const PointXYZI &p)
		{
			x = p.x; y = p.y; z = p.z;
			intensity = p.intensity;
		}

		inline PointXYZI ()
		{
			x = y = z = 0.0f;

			intensity = 0.0f;
		}
		inline PointXYZI (float _intensity)
		{
			x = y = z = 0.0f;

			intensity = _intensity;
		}

		float x;
		float y;
		float z;

		float intensity;
	};

	struct PointXYZL
	{
		inline PointXYZL (const PointXYZL &p)
		{
			x = p.x; y = p.y; z = p.z;
			label = p.label;
		}

		inline PointXYZL ()
		{
			x = y = z = 0.0f;
			label = 0;
		}

		float x;
		float y;
		float z;

		int label;
	};

	struct PointXYZILR
	{
		inline PointXYZILR (const PointXYZILR &p)
		{
			x = p.x; y = p.y; z = p.z;
			label = p.label;

			intensity = p.intensity;
			return_num = p.return_num;
			num_return = p.num_return;
		}

		inline PointXYZILR ()
		{
			x = y = z = 0.0f;
			label = 0;
			intensity = 0.0f;
			return_num = 0;
			num_return = 0;
		}

		float x;
		float y;
		float z;

		float intensity;
		int label;

		int return_num;
		int num_return;
	};

	struct PointXYZRGB
	{
		inline PointXYZRGB (const PointXYZRGB &p)
		{
			x = p.x; y = p.y; z = p.z;

			r = p.r; y = p.g; b = p.b; a = p.a;
		}

		inline PointXYZRGB ()
		{
			x = y = z = 0.0f;
			r = g = b = a = 0;
		}

		float x;
		float y;
		float z;

		unsigned char r;
		unsigned char g;
		unsigned char b;
		unsigned char a;
	};
}

#endif
