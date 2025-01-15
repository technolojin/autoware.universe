# autoware_architect_get_module_list.cmake

function(autoware_architect_get_module_list)
  # define the path to the shared file
  set(shared_file "${CMAKE_BINARY_DIR}/../autoware_perception_architect/autoware_architect_yaml_filelist.txt")

  # check if the shared file exists
  if(EXISTS ${shared_file})
    # read the content of the shared file
    file(READ ${shared_file} yaml_files_content)

    # print the final list of yaml files
    message("Final autoware_architect_yaml_filelist:\n${yaml_files_content}")
  else()
    message("No YAML files collected.")
  endif()
endfunction()
