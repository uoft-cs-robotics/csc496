import numpy as np
import stltovoxel
import pydicom
from pydicom.dataset import Dataset, FileDataset
from pydicom.uid import generate_uid, ExplicitVRLittleEndian
import os
from stl import mesh

def convert_stl_to_volume(numpy_stl_mesh, resolution=100, voxel_size=None, parallel=False):
    """Convert STL to voxel volume using stltovoxel"""
    meshes = [np.hstack(
        (numpy_stl_mesh.v0[:, np.newaxis], numpy_stl_mesh.v1[:, np.newaxis], numpy_stl_mesh.v2[:, np.newaxis])
    )]
    vol, scale, shift = stltovoxel.convert_meshes(meshes, resolution, voxel_size, parallel)
    return np.transpose(vol, (2, 1, 0)).astype(np.uint16)  # Transpose for DICOM orientation

def voxel_to_dicom(voxel_data, output_dir, voxel_size=1.0):
    """Convert 3D voxel array to DICOM series"""
    os.makedirs(output_dir, exist_ok=True)
    
    # Create base DICOM dataset
    ds = FileDataset(None, {}, preamble=b"\0"*128)
    ds.file_meta = Dataset()
    ds.file_meta.TransferSyntaxUID = ExplicitVRLittleEndian
    ds.file_meta.MediaStorageSOPClassUID = '1.2.840.10008.5.1.4.1.1.2'  # CT
    
    # Common DICOM headers
    ds.PatientName = "STL Conversion"
    ds.PatientID = "STL2DCM"
    ds.Modality = "CT"
    ds.SamplesPerPixel = 1
    ds.PhotometricInterpretation = "MONOCHROME2"
    ds.PixelSpacing = [voxel_size, voxel_size]
    ds.SliceThickness = voxel_size
    ds.ImageOrientationPatient = [1, 0, 0, 0, 1, 0]  # Axial orientation
    ds.RescaleIntercept = 0
    ds.RescaleSlope = 1.0
    ds.BitsAllocated = 16
    ds.BitsStored = 16
    ds.HighBit = 15
    ds.PixelRepresentation = 0
    
    # Generate shared UIDs
    study_uid = generate_uid()
    series_uid = generate_uid()
    
    # Create slices
    for z in range(voxel_data.shape[0]):
        ds.InstanceNumber = z + 1
        ds.ImagePositionPatient = [0.0, 0.0, z * voxel_size]
        ds.SOPInstanceUID = generate_uid()
        ds.StudyInstanceUID = study_uid
        ds.SeriesInstanceUID = series_uid
        ds.Rows, ds.Columns = voxel_data.shape[1], voxel_data.shape[2]
        
        # Convert binary voxels to DICOM pixel data (0-65535)
        slice_data = (voxel_data[z] * 65535).astype(np.uint16)
        ds.PixelData = slice_data.tobytes()
        
        ds.save_as(os.path.join(output_dir, f'slice_{z:04d}.dcm'))

def stl_to_dicom(stl_path, output_dir, resolution=100, voxel_size=1.0):
    """Main conversion workflow"""
    # Load STL
    stl_mesh = mesh.Mesh.from_file(stl_path)
    
    # Convert to voxels
    voxel_volume = convert_stl_to_volume(
        stl_mesh,
        resolution=resolution,
        voxel_size=voxel_size
    )
    
    # Convert to DICOM
    voxel_to_dicom(
        voxel_volume,
        output_dir,
        voxel_size=voxel_size
    )

if __name__ == "__main__":
    # Change paths
    stl_to_dicom(
        stl_path="/Users/nirmalpol/PycharmProjects/csc496/scratchpad/stl2dicom/mixel-motor-holders.STL",
        output_dir="/Users/nirmalpol/PycharmProjects/csc496/scratchpad/stl2dicom/dicom_output",
        resolution=200,  # Voxel grid resolution
        voxel_size=0.5   # Physical size per voxel in mm
    )