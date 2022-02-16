// This software is in the public domain. Where that dedication is not
// recognized, you are granted a perpetual, irrevocable license to copy,
// distribute, and modify this file as you see fit.
// Authored in 2015 by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
// https://github.com/ddiakopoulos/tinyply

#include <thread>
#include <chrono>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

#include "tinyply.h"

using namespace tinyply;

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timepoint;
std::chrono::high_resolution_clock c;

inline std::chrono::time_point<std::chrono::high_resolution_clock> now()
{
	return c.now();
}

inline double difference_micros(timepoint start, timepoint end)
{
	return (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

void write_ply_example(const std::string & filename)
{
	std::vector<float> verts;
	std::vector<float> norms;
	std::vector<uint8_t> colors;

	std::vector<int32_t> vertexIndicies;
	std::vector<float> faceTexcoords;

	// Per-vertex elements
	verts = {
		0.f, -100.462f, -142.5f,
		123.409f, -100.462f, 71.25f,
		0.f, 100.462f, 0.f,
		-123.409f, -100.462f, 71.25f,
		0.f, -100.462f, -142.5f,
		0.f, -100.462f, -142.5f,
		123.409f, -100.462f, 71.25f,
		123.409f, -100.462f, 71.25f,
		0.f, 100.462f, 0.f,
		0.f, 100.462f, 0.f,
		-123.409f, -100.462f, 71.25f,
		-123.409f, -100.462f, 71.25f
	};

	norms = {
		0.853811f, 0.349609f, -0.492948f,
		0.853811f, 0.349609f, -0.492948f,
		0.0f, 0.350761f, 0.989145f,
		0.0f, 0.349609f, 0.985896f,
		-0.853811f, 0.349609f, -0.492948f,
		0.0f, -1.0472f, 0.0f,
		0.0f, 0.349609f, 0.985896f,
		0.0f, -1.0472f, 0.0f,
		0.856624f, 0.350761f, -0.494572f,
		-0.856624f, 0.350761f, -0.494572f,
		-0.853811f, 0.349609f, -0.492948f,
		0.0f, -1.0472f, 0.0f
	};

	colors = {
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 192, 192, 255,
		192, 193, 194, 255,
		195, 196, 197, 255
	};

	// Per-face elements
	vertexIndicies = { 6, 2, 3, 0, 8, 1, 10, 9, 4, 5, 7, 11 };
	faceTexcoords = {
		0.199362f, 0.679351f, 0.399522f, 
		0.333583f, 0.599682f, 0.679351f,
		0.000000f, 0.332206f, 0.399522f,
		0.333583f, 0.199362f, 0.679351f, 
		0.599682f, 0.679351f, 0.399522f,
		0.333583f, 0.799044f, 0.332206f, 
		0.799044f, 0.332206f, 1.000000f,
		0.678432f, 0.599682f, 0.679351f 
	};

	// Tinyply does not perform any file i/o internally
	std::filebuf fb;
	//fb.open(filename, std::ios::out | std::ios::binary);
	fb.open(filename, std::ios::out | std::ios::binary);
	std::ostream outputStream(&fb);

	PlyFile myFile;

	myFile.add_properties_to_element("vertex", { "x", "y", "z" }, verts);
	myFile.add_properties_to_element("vertex", { "nx", "ny", "nz" }, norms);
	myFile.add_properties_to_element("vertex", { "red", "green", "blue", "alpha" }, colors);

	// List property types must also be created with a count and type of the list (data property type
	// is automatically inferred from the type of the vector argument). 
	myFile.add_properties_to_element("face", { "vertex_indices" }, vertexIndicies, 3, PlyProperty::Type::UINT8);
	myFile.add_properties_to_element("face", { "texcoord" }, faceTexcoords, 6, PlyProperty::Type::UINT8);

	myFile.comments.push_back("generated by tinyply");
	myFile.write(outputStream, /*true*/false);

	fb.close();
}

void read_ply_file(const std::string & filename)
{
	// Tinyply can and will throw exceptions at you!
	try
	{
		// Read the file and create a std::istringstream suitable
		// for the lib -- tinyply does not perform any file i/o.
		std::ifstream ss(filename, std::ios::binary);

		// Parse the ASCII header fields
		PlyFile file(ss);

		for (auto e : file.get_elements())
		{
			std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
			for (auto p : e.properties)
			{
				std::cout << "\tproperty - " << p.name << " (" << PropertyTable[p.propertyType].str << ")" << std::endl;
			}
		}
		std::cout << std::endl;

		for (auto c : file.comments)
		{
			std::cout << "Comment: " << c << std::endl;
		}

		// Define containers to hold the extracted data. The type must match
		// the property type given in the header. Tinyply will interally allocate the
		// the appropriate amount of memory.
		std::vector<float> verts;
		std::vector<float> norms;
		std::vector<uint8_t> colors;

		std::vector<uint32_t> faces;
		std::vector<float> uvCoords;

		uint32_t vertexCount, normalCount, colorCount, faceCount, faceTexcoordCount, faceColorCount;
		vertexCount = normalCount = colorCount = faceCount = faceTexcoordCount = faceColorCount = 0;

		// The count returns the number of instances of the property group. The vectors
		// above will be resized into a multiple of the property group size as
		// they are "flattened"... i.e. verts = {x, y, z, x, y, z, ...}
		vertexCount = file.request_properties_from_element("vertex", { "x", "y", "z" }, verts);
		normalCount = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }, norms);
		colorCount = file.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" }, colors);

		// For properties that are list types, it is possibly to specify the expected count (ideal if a
		// consumer of this library knows the layout of their format a-priori). Otherwise, tinyply
		// defers allocation of memory until the first instance of the property has been found
		// as implemented in file.read(ss)
		faceCount = file.request_properties_from_element("face", { "vertex_indices" }, faces, 3);
		faceTexcoordCount = file.request_properties_from_element("face", { "texcoord" }, uvCoords, 6);

		// Now populate the vectors...
		timepoint before = now();
		file.read(ss);
		timepoint after = now();

		// Good place to put a breakpoint!
		std::cout << "Parsing took " << difference_micros(before, after) << "μs: " << std::endl;
		std::cout << "\tRead " << verts.size() << " total vertices (" << vertexCount << " properties)." << std::endl;
		std::cout << "\tRead " << norms.size() << " total normals (" << normalCount << " properties)." << std::endl;
		std::cout << "\tRead " << colors.size() << " total vertex colors (" << colorCount << " properties)." << std::endl;
		std::cout << "\tRead " << faces.size() << " total faces (triangles) (" << faceCount << " properties)." << std::endl;
		std::cout << "\tRead " << uvCoords.size() << " total texcoords (" << faceTexcoordCount << " properties)." << std::endl;
		
		/*
		// Fixme - tinyply isn't particularly sensitive to mismatched properties and prefers to crash instead of throw. Use
		// actual data from parsed headers instead of manually defining properties added to a new file like below:

		std::filebuf fb;
		fb.open("converted.ply", std::ios::out | std::ios::binary);
		std::ostream outputStream(&fb);

		PlyFile myFile;

		myFile.add_properties_to_element("vertex", { "x", "y", "z" }, verts);
		myFile.add_properties_to_element("vertex", { "red", "green", "blue" }, colors);
		myFile.add_properties_to_element("face", { "vertex_indices" }, faces, 3, PlyProperty::Type::UINT8);

		myFile.comments.push_back("generated by tinyply");
		myFile.write(outputStream, true);

		fb.close();
		*/
	}

	catch (const std::exception & e)
	{
		std::cerr << "Caught exception: " << e.what() << std::endl;
	}
}

int main(int argc, char *argv[])
{
	write_ply_example("example_tetrahedron.ply");
	read_ply_file("example_tetrahedron.ply");
	return 0;
}
