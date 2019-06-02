#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tarfile
import ast
import argparse
import os
import sys

# Import Google Libraries
from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive
from pydrive.files import GoogleDriveFileList
import googleapiclient.errors


def get_folder_id(drive, parent, folder):
    try:
        file_list = drive.ListFile({
            'q':
            "'{0}' in parents and trashed=false".format(parent)
        }).GetList()
        # Exit if the parent folder doesn't exist
    except googleapiclient.errors.HttpError as err:
        # Parse error message
        message = ast.literal_eval(err.content)['error']['message']
        if message == 'File not found: ':
            print(message + folder)
            exit(1)
        else:
            raise

    # Find the the destination folder in the parent folder's files
    for file1 in file_list:
        if file1['title'] == folder:
            print('title: %s, id: %s' % (file1['title'], file1['id']))
            return file1['id']


def create_folder(drive, folder_name, parent_id):

    folder_metadata = {
        'title': folder_name,
        'mimeType': 'application/vnd.google-apps.folder',
        'parents': [{
            "kind": "drive#fileLink",
            "id": parent_id
        }]
    }

    folder = drive.CreateFile(folder_metadata)
    folder.Upload()
    print("Created file {} with mimeType {}".format(folder['title'],
                                                    folder['mimeType']))


def upload_file(drive, file, parent_id):

    print("Uploading {}".format(file))

    f = drive.CreateFile({
        "parents": [{
            "kind": "drive#fileLink",
            "id": parent_id
        }]
    })
    f.SetContentFile(file)
    f.Upload()


# Main
if __name__ == '__main__':

    # Import arguments
    parser = argparse.ArgumentParser(description='Upload a dataset to Google Drive')
    parser.add_argument(
        '--path', action="store", help="File Path of Folder Dataset")

    args = parser.parse_args()

    file_path = args.path

    # Authenticate with Google Drive
    gauth = GoogleAuth()

    # Try to load saved client credentials
    gauth.LoadCredentialsFile("mycreds.txt")
    if gauth.credentials is None:
        # Authenticate if they're not there
        gauth.LocalWebserverAuth()
    #elif gauth.access_token_expired:
        # Refresh them if expired
        #gauth.Refresh()
        #else:
        # Initialize the saved creds
        #gauth.Authorize()

    # Save the current credentials to a file
    gauth.SaveCredentialsFile("mycreds.txt")

    drive = GoogleDrive(gauth)

    # Auto-iterate through all files in the parent folder.
    file_list = GoogleDriveFileList()

    # Relevent folder IDs
    # research -> 1HUks78Wgg5JS9sLfyDD-VLLrR1fn8ETi
    # diy_driverless_car_ROS -> 1WHX__MM692Xczvzu1hi94W9a-HQIATMW
    # rover_ml -> 1qrt1dFuGdKg2JILzJXHkn8wIDmoCKj7T
    # output -> 1US1efrQQH9N9TNA5Nsp2tbnEkVFD2ViD
    # walker

    parent_id = '1qrt1dFuGdKg2JILzJXHkn8wIDmoCKj7T'
    folder_name = "output"

    folder_id = get_folder_id(drive, parent_id, folder_name)

    #create_folder(drive, 'test', parent)

    # Archive dataset folder
    tar_file = file_path.split("/")
    tar_file_base = tar_file[len(tar_file) - 1]
    tar_file = tar_file_base + ".tar.gz"
    tf = tarfile.open(tar_file, mode="w:gz")
    tf.add(file_path, arcname=tar_file_base)
    tf.close()
    print("\nCreated {} \n".format(tar_file))

    # Upload to Google Drive
    upload_file(drive, tar_file, folder_id)
    print("\nUploaded {}".format(tar_file))

    # Move to destination folder
    output = file_path.split("/")
    output = '/'.join(output[0:2])
    output = output + "/" + tar_file

    print("Moving {} to {}\n".format(tar_file, output))

    os.rename(tar_file, output)
