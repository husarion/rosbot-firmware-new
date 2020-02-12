#!/usr/bin/env python3

import os
import sys
import json
import argparse

def getJetsonData(filename):
    try:
        with open(filename,'r') as json_file:
            data = json.load(json_file)
    except IOError:
        print("Cannot locate %s file:" % filename)
        return None
    else:
        print("File %s found!" % filename)
        return data

def main():
    parser = argparse.ArgumentParser(description='This program updates symbols in c_cpp_properties.json file.')
    parser.add_argument('-t', '--type', type=str, default='release', help='selects type of build', choices=['release', 'debug'])
    parser.add_argument('-v', '--verbose', action='store_true', help='enable verbose output')
    args = parser.parse_args()

    if(args.type == 'release'):
        profile_cxx_file_path = os.path.join(os.path.dirname(__file__),'BUILD','RELEASE','.profile-cxx')
    else:
        profile_cxx_file_path = os.path.join(os.path.dirname(__file__),'BUILD','DEBUG','.profile-cxx')

    c_cpp_properties_file_path = os.path.join(os.path.dirname(__file__),'.vscode','c_cpp_properties.json')
    
    c_cpp_properties_json = getJetsonData(c_cpp_properties_file_path)
    old_symbols = c_cpp_properties_json['env']['DefaultDefines']
    if(args.verbose):
        print("Old symbols:\n%s" % (json.dumps(old_symbols,indent=4)))

    new_symbols = getJetsonData(profile_cxx_file_path)['symbols']
    if(args.verbose):
        print("New symbols:\n%s" % (json.dumps(new_symbols,indent=4)))

    c_cpp_properties_json['env']['DefaultDefines'] = new_symbols 

    with open(c_cpp_properties_file_path, 'w') as out_file:
        json.dump(c_cpp_properties_json, out_file, indent=4)

    print('Symbols updated successfully.')

if __name__ == '__main__':
    main()