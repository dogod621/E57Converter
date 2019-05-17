# E57Converter
Convert E57 point cloud format to PCL pcd format 

# Submodules
	libE57Format: https://github.com/asmaloney/libE57Format

# Dependency
	PCL: https://github.com/PointCloudLibrary/pcl
	Xerces-C (for libE57Format):https://github.com/apache/xerces-c

# How to build
	1. clone with submodul: git clone --recursive https://github.com/dogod621/E57Converter.git
	2. make the project: use CMAKE (PS. you may want to set USING_STATIC_XERCES ON)
	
# How to use
	For example: 
		you want to convert a .e57 file - "D:/src.e57" to PCL .pcd file - "D:/dst.pcd"
		
		1. Convert .e57 to PCL OutOfCoreOctree: 
			Command:
				E57Converter.exe -convert -src "D:/src.e57" -dst "D:/dst/" -res 2 -min "-20 -20 -20" -max "20 20 20"
		
			Paramerte description:
				-convert:
					specify you are doing a conversion.
				
				-src:
					input file.
					
				-dst
					output file, for this example is a not exist folder ( You must give a not exist folder for create PCL OutOfCoreOctree ).
					
				-res 2: 
					means node dimension of the octree is 2 meters.
					
				-min "-20 -20 -20": 
					means AABB xyz of octree minimum is -20 meter.
					
				-max "20 20 20": 
					means AABB xyz of octree maximum is 20 meter.
					
				-samplePercent 
					sets the sampling percent for constructing LODs.
					(if not given, default is 0.125.)
					
		2. Convert PCL OutOfCoreOctree to .pcd:
			Command:
				E57Converter.exe -convert -src "D:/dst/" -dst "D:/dst.pcd" -voxelUnit 0.005
				
			Paramerte description:
				-convert:
					specify you are doing a conversion
				
				-src:
					input file, for this example is the PCL OutOfCoreOctree folder.
					
				-dst:
					output file.
					
				-voxelUnit:
					the voxel filter voxel size, for this exammple is 0.005 meter (0.5 cm)
					(For removing duplicate scan points or for downsampling)

# Useful fuctions:
	1. Print .e57 file - "D:/src.e57" tree structure (This is useful for e57 developers):
		Command:
			E57Converter.exe -printE57Format -src "D:/src.e57"
		
		Paramerte description:
			-printE57Format:
				specify you want to print .e57 file tree structure.
				
			-src:
				the .e57 file
				
	2. Convert .pcd file - "D:/src.pcd" to .ply file - "D:/dst.ply": 
		(This is different from PCL tool pcd2ply, this will not output point's scan index and intensity, because PLY file users may not want those values )
		Command:
			E57Converter.exe -convert -src "D:/src.pcd" -dst "D:/dst.ply" -rgb
			
			-convert:
				specify you are doing a conversion
				
			-src:
				input file,.
				
			-dst:
				output file.
				
			-rgb:
				specify output point color.
				
			-camera:
				specify output camera (default false)
			
			-binary:
				specify output as binary (default false)
			
	3. Rebuild PCL OutOfCoreOctree - "D:/src/" LOD:
		Command:
			E57Converter.exe -buildLOD -src "D:/src/" -samplePercent 0.125
	
		Paramerte description:
			-buildLOD:
				specify you want to rebuild PCL OutOfCoreOctree LOD.
				
			-src:
				the PCL OutOfCoreOctree folder.
				
			-samplePercent 
				sets the sampling percent for constructing LODs.
				
			
	
	
		


