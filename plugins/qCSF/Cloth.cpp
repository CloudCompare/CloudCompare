#include "Cloth.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>

//system
#include <assert.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <queue>

Cloth::Cloth(	const Vec3& _origin_pos,
				int _num_particles_width,
				int _num_particles_height,
				double _step_x,
				double _step_y,
				double _smoothThreshold,
				double _heightThreshold,
				int rigidness,
				double time_step)
	: constraint_iterations(rigidness)
	, time_step(time_step)
	, smoothThreshold(_smoothThreshold)
	, heightThreshold(_heightThreshold)
	, num_particles_width(_num_particles_width)
	, num_particles_height(_num_particles_height)
	, origin_pos(_origin_pos)
	, step_x(_step_x)
	, step_y(_step_y)
{
	particles.resize(num_particles_width*num_particles_height); //I am essentially using this vector as an array with room for num_particles_width*num_particles_height particles

	double time_step2 = time_step*time_step;

	// creating particles in a grid
	for (int i = 0; i < num_particles_width; i++)
	{
		for (int j = 0; j < num_particles_height; j++)
		{
			Vec3 pos(	origin_pos.x + i * step_x,
						origin_pos.y,
						origin_pos.z + j * step_y);

			particles[j*num_particles_width + i] = Particle(pos, time_step2); // insert particle in column i at j'th row
			particles[j*num_particles_width + i].pos_x = i;
			particles[j*num_particles_width + i].pos_y = j;
		}
	}

	// Connecting immediate neighbor particles with constraints (distance 1 and sqrt(2) in the grid)
	for (int x = 0; x<num_particles_width; x++)
	{
		for (int y = 0; y<num_particles_height; y++)
		{
			if (x < num_particles_width - 1)
			{
				addConstraint(&getParticle(x, y), &getParticle(x + 1, y));
			}

			if (y < num_particles_height - 1)
			{
				addConstraint(&getParticle(x, y), &getParticle(x, y + 1));
			}

			if (x < num_particles_width - 1 && y < num_particles_height - 1)
			{
				addConstraint(&getParticle(x, y), &getParticle(x + 1, y + 1));
				addConstraint(&getParticle(x + 1, y), &getParticle(x, y + 1));
			}
		}
	}

	// Connecting secondary neighbors with constraints (distance 2 and sqrt(4) in the grid)
	for (int x = 0; x < num_particles_width; x++)
	{
		for (int y = 0; y < num_particles_height; y++)
		{
			if (x < num_particles_width - 2)
			{
				addConstraint(&getParticle(x, y), &getParticle(x + 2, y));
			}


			if (y < num_particles_height - 2)
			{
				addConstraint(&getParticle(x, y), &getParticle(x, y + 2));
			}


			if (x < num_particles_width - 2 && y < num_particles_height - 2)
			{
				addConstraint(&getParticle(x, y), &getParticle(x + 2, y + 2));
				addConstraint(&getParticle(x + 2, y), &getParticle(x, y + 2));
			}
		}
	}
}

ccMesh* Cloth::toMesh() const
{
	ccPointCloud* vertices = new ccPointCloud("vertices");
	ccMesh* mesh = new ccMesh(vertices);
	mesh->addChild(vertices);
	vertices->setEnabled(false);
	unsigned vertCount = static_cast<unsigned>(getSize());
	unsigned triCount = static_cast<unsigned>((num_particles_height - 1) * (num_particles_width - 1) * 2);
	if (!vertices->reserve(vertCount)
		|| !mesh->reserve(triCount))
	{
		//not enough memory to generate the cloth mesh
		delete mesh;
		mesh = 0;
		return 0;
	}

	//copy the vertices (particles)
	for (int i = 0; i < getSize(); ++i)
	{
		const Particle& particle = particles[i];
		vertices->addPoint(CCVector3(	static_cast<PointCoordinateType>(particle.pos.x),
										static_cast<PointCoordinateType>(particle.pos.z),
										static_cast<PointCoordinateType>(-particle.pos.y)));
	}

	//and create the triangles
	for (int x = 0; x < num_particles_width - 1; ++x)
	{
		for (int y = 0; y < num_particles_height-1; ++y)
		{
			// A ---------- B
			// |            |
			// D ---------- C
			int iA = y * num_particles_width + x;
			int iD = iA + num_particles_width;
			int iB = iA + 1;
			int iC = iD + 1;
			mesh->addTriangle(iA, iB, iD);
			mesh->addTriangle(iD, iB, iC);
		}
	}

	return mesh;
}

double Cloth::timeStep()
{
	int particleCount = static_cast<int>(particles.size());

#pragma omp parallel for
	for (int i = 0; i < particleCount; i++)
	{
		particles[i].timeStep();
	}
/*
Instead of interating over all the constraints several times, we 
compute the overall displacement of a particle accroding to the rigidness
*/

#pragma omp parallel for
	for (int j = 0; j < particleCount; j++)
	{
		particles[j].satisfyConstraintSelf(constraint_iterations);
	}


//	for (int i = 0; i<constraint_iterations; i++) // iterate over all constraints several times
//	{
//#pragma omp parallel for
//		for (int j = 0; j < constraints.size(); j++)
//		{
//			constraints[j].satisfyConstraint();
//		}
//	}

	double maxDiff = 0;
#pragma omp parallel for
	for (int i = 0; i < particleCount; i++)
	{
		if (particles[i].isMovable())
		{
			double diff = fabs(particles[i].old_pos.y - particles[i].pos.y);
			if (diff > maxDiff)
				maxDiff = diff;
		}
	}

	return maxDiff;
}

void Cloth::addForce(const Vec3& direction)
{
	int particleCount = static_cast<int>(particles.size());

	// add the forces to each particle
#pragma omp parallel for
	for (int i = 0; i < particleCount; i++)
	{
		particles[i].addForce(direction);
	}
}

//testing the collision
void Cloth::terrainCollision()
{
	assert(particles.size() == heightvals.size());

	int particleCount = static_cast<int>(particles.size());
#pragma omp parallel for
	for (int i = 0; i < particleCount; i++)
	{
		Particle& particle = particles[i];
		if (particle.pos.y < heightvals[i]) // if the particle is inside the ball
		{
			particle.offsetPos(Vec3(0, heightvals[i] - particle.pos.y, 0));
			particle.makeUnmovable();
		}
	}
}

void Cloth::movableFilter()
{
	for (int x = 0; x < num_particles_width; x++)
	{
		for (int y = 0; y < num_particles_height; y++)
		{
			Particle& ptc = getParticle(x, y);
			if (ptc.isMovable() && !ptc.isVisited)
			{
				std::queue<int> que;
				std::vector<XY> connected; //store the connected component
				std::vector< std::vector<int> > neibors;
				int sum = 1;
				int index = y*num_particles_width + x;
				// visit the init node
				connected.push_back(XY(x,y));
				particles[index].isVisited = true;
				//enqueue the init node
				que.push(index);
				while (!que.empty())
				{
					Particle& ptc_f = particles[que.front()];
					que.pop();
					int cur_x = ptc_f.pos_x;
					int cur_y = ptc_f.pos_y;
					std::vector<int> neighbor;

					if (cur_x > 0)
					{
						Particle& ptc_left = getParticle(cur_x - 1, cur_y);
						if (ptc_left.isMovable())
						{
							if (!ptc_left.isVisited)
							{
								sum++;
								ptc_left.isVisited = true;
								connected.push_back(XY(cur_x - 1, cur_y));
								que.push(num_particles_width*cur_y + cur_x - 1);
								neighbor.push_back(sum - 1);
								ptc_left.c_pos = sum - 1;
							}
							else
							{
								neighbor.push_back(ptc_left.c_pos);
							}
						}
					}

					if (cur_x < num_particles_width - 1)
					{
						Particle& ptc_right = getParticle(cur_x + 1, cur_y);
						if (ptc_right.isMovable())
						{
							if (!ptc_right.isVisited)
							{
								sum++;
								ptc_right.isVisited = true;
								connected.push_back(XY(cur_x + 1, cur_y));
								que.push(num_particles_width*cur_y + cur_x + 1);
								neighbor.push_back(sum - 1);
								ptc_right.c_pos = sum - 1;
							}
							else
							{
								neighbor.push_back(ptc_right.c_pos);
							}
						}
					}

					if (cur_y > 0)
					{
						Particle& ptc_bottom = getParticle(cur_x, cur_y - 1);
						if (ptc_bottom.isMovable())
						{
							if (!ptc_bottom.isVisited)
							{
								sum++;
								ptc_bottom.isVisited = true;
								connected.push_back(XY(cur_x, cur_y - 1));
								que.push(num_particles_width*(cur_y - 1) + cur_x);
								neighbor.push_back(sum - 1);
								ptc_bottom.c_pos = sum - 1;
							}
							else
							{
								neighbor.push_back(ptc_bottom.c_pos);
							}
						}
					}

					if (cur_y < num_particles_height - 1)
					{
						Particle& ptc_top = getParticle(cur_x, cur_y + 1);
						if (ptc_top.isMovable())
						{
							if (!ptc_top.isVisited)
							{
								sum++;
								ptc_top.isVisited = true;
								connected.push_back(XY(cur_x, cur_y + 1));
								que.push(num_particles_width*(cur_y + 1) + cur_x);
								neighbor.push_back(sum - 1);
								ptc_top.c_pos = sum - 1;
							}
							else
							{
								neighbor.push_back(ptc_top.c_pos);
							}
						}
					}
					neibors.push_back(neighbor);
				}

				//Slope postprocessing
				if (sum > 100)
				{
					std::vector<int> edgePoints;
					findUnmovablePoint(connected, heightvals, edgePoints);
					handle_slop_connected(edgePoints, connected, neibors, heightvals);
				}
			}
		}
	}
}

void Cloth::findUnmovablePoint(	const std::vector<XY>& connected,
								const std::vector<double>& heightvals,
								std::vector<int>& edgePoints)
{
	for (size_t i = 0; i < connected.size(); i++)
	{
		int x = connected[i].x;
		int y = connected[i].y;
		int index = y*num_particles_width + x;
		Particle& ptc = getParticle(x, y);
		if (x > 0)
		{
			const Particle& ptc_x = getParticle(x - 1, y);
			if (!ptc_x.isMovable())
			{
				int index_ref = y*num_particles_width + x - 1;
				if (fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold && ptc.pos.y - heightvals[index] < heightThreshold)
				{
					Vec3 offsetVec(0, heightvals[index] - ptc.pos.y, 0);
					particles[index].offsetPos(offsetVec);
					ptc.makeUnmovable();
					edgePoints.push_back(static_cast<int>(i));
					continue;
				}
			}
		}

		if (x < num_particles_width - 1)
		{
			const Particle& ptc_x = getParticle(x + 1, y);
			if (!ptc_x.isMovable())
			{
				int index_ref = y*num_particles_width + x + 1;
				if (fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold && ptc.pos.y - heightvals[index] < heightThreshold)
				{
					Vec3 offsetVec(0, heightvals[index] - ptc.pos.y, 0);
					particles[index].offsetPos(offsetVec);
					ptc.makeUnmovable();
					edgePoints.push_back(static_cast<int>(i));
					continue;
				}
			}
		}

		if (y > 0)
		{
			const Particle& ptc_y = getParticle(x, y - 1);
			if (!ptc_y.isMovable())
			{
				int index_ref = (y - 1)*num_particles_width + x;
				if (fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold && ptc.pos.y - heightvals[index] < heightThreshold)
				{
					Vec3 offsetVec(0, heightvals[index] - ptc.pos.y, 0);
					particles[index].offsetPos(offsetVec);
					ptc.makeUnmovable();
					edgePoints.push_back(static_cast<int>(i));
					continue;
				}
			}

		}

		if (y < num_particles_height - 1)
		{
			const Particle& ptc_y = getParticle(x, y + 1);
			if (!ptc_y.isMovable())
			{
				int index_ref = (y + 1)*num_particles_width + x;
				if (fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold && ptc.pos.y - heightvals[index] < heightThreshold)
				{
					Vec3 offsetVec(0, heightvals[index] - ptc.pos.y, 0);
					particles[index].offsetPos(offsetVec);
					ptc.makeUnmovable();
					edgePoints.push_back(static_cast<int>(i));
					continue;
				}
			}
		}
	}
}

//implementing postprocessing to every group of movable points
void Cloth::handle_slop_connected(	const std::vector<int>& edgePoints,
									const std::vector<XY>& connected,
									const std::vector< std::vector<int> >& neibors,
									const std::vector<double>& heightvals)
{
	std::vector<bool> visited(connected.size(), false);

	std::queue<int> que;
	for (size_t i = 0; i < edgePoints.size(); i++)
	{
		que.push(edgePoints[i]);
		visited[edgePoints[i]] = true;
	}

	while (!que.empty())
	{
		int index = que.front();
		que.pop();
		//ÅÐ¶ÏÖÜ±ßµãÊÇ·ñÐèÒª´¦Àí
		int index_center = connected[index].y*num_particles_width + connected[index].x;
		for (size_t i = 0; i < neibors[index].size(); i++)
		{
			int index_neibor = connected[neibors[index][i]].y*num_particles_width + connected[neibors[index][i]].x;
			if (fabs(heightvals[index_center] - heightvals[index_neibor]) < smoothThreshold && fabs(particles[index_neibor].pos.y - heightvals[index_neibor]) < heightThreshold)
			{
				Vec3 offsetVec(0, heightvals[index_neibor] - particles[index_neibor].pos.y, 0);
				particles[index_neibor].offsetPos(offsetVec);
				particles[index_neibor].makeUnmovable();
				if (visited[neibors[index][i]] == false)
				{
					que.push(neibors[index][i]);
					visited[neibors[index][i]] = true;
				}
			}
		}
	}
}

void Cloth::saveToFile(std::string path)
{
	std::string filepath = "cloth_nodes.txt";
	if (path == "")
	{
		filepath = "cloth_nodes.txt";
	}
	else
	{
		filepath = path;
	}
	std::ofstream f1(filepath);
	if (!f1)
		return;
	for (size_t i = 0; i < particles.size(); i++)
	{
		//if (!particles[i].isMovable())
		f1 << std::fixed << std::setprecision(8) << particles[i].pos.x << "	" << particles[i].pos.z << "	" << -particles[i].pos.y << std::endl;
	}
	f1.close();
}

void Cloth::saveMovableToFile(std::string path)
{
	std::string filepath = "cloth_movable.txt";
	if (path == "")
	{
		filepath = "cloth_movable.txt";
	}
	else
	{
		filepath = path;
	}
	std::ofstream f1(filepath);
	if (!f1)
		return;
	for (size_t i = 0; i < particles.size(); i++)
	{
		if (particles[i].isMovable())
			f1 << std::fixed << std::setprecision(8) << particles[i].pos.x << "	" << particles[i].pos.z << "	" << -particles[i].pos.y << std::endl;
	}
	f1.close();
}
