current_branch=$(git rev-parse --abbrev-ref HEAD)
pdoc \
--template-directory ./custom-template/ \
--logo ../images/banner.png \
--logo-link https://github.com/hello-robot/stretch_body \
--favicon ../images/hello_robot_favicon.png \
--edit-url stretch_body=https://github.com/hello-robot/stretch_body/tree/$current_branch/body/stretch_body/ \
stretch_body -o . 