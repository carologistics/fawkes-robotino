#! /bin/bash
# Automated generation of data for the lookup plugin for grasping skills used
# in the rcll.
# Old data is deleted when executing this script.

data_dir=${FAWKES_DIR}/data
gauss_fit=${FAWKES_DIR}/bin/./gaussian_mixture_fit.py
sample_script="${FAWKES_DIR}/bin/./mongodb_skillsim_lookup.py generate"
gauss_fit_options="--error-criterion BIC --plot --max 6"
skillsim_options="-n 1000"

echo -e "wp-put"
gauss_params=$(${gauss_fit}  \
	--csv "${data_dir}/wp-put.csv" ${gauss_fit_options} )

${sample_script} -dc ${skillsim_options} -s bring_product_to \
	-a place ${gauss_params} -l 10 -u 45 --inject-failure "0.05" "Product Dropped" > /dev/null 2>&1

echo -e "wp-put-slide-cc"
gauss_params=$(${gauss_fit} \
	--csv "${data_dir}/wp-put-slide-cc.csv" ${gauss_fit_options} )

${sample_script} ${skillsim_options} -s bring_product_to \
	-a place -a slide TRUE ${gauss_params} -l 10 -u 45 --inject-failure "0.05" "Product Dropped" > /dev/null 2>&1

echo -e "wp-get"
gauss_params=$(${gauss_fit} \
	--csv "${data_dir}/wp-get.csv" ${gauss_fit_options} )

${sample_script} ${skillsim_options} -s get_product_from \
	-a place -a side ${gauss_params} -l 10 -u 45 --inject-failure "0.05" "Product Dropped" > /dev/null 2>&1

echo -e "wp-get-shelf"

gauss_params=$(${gauss_fit} \
	--csv "${data_dir}/wp-get-shelf.csv" ${gauss_fit_options} )

${sample_script} ${skillsim_options} -s get_product_from \
	-a place -a shelf ${gauss_params} -l 10 -u 45 --inject-failure "0.05" "Product Dropped" > /dev/null 2>&1

echo -e "wp-discard"

gauss_params=$(${gauss_fit} \
	--csv "${data_dir}/wp-discard.csv" ${gauss_fit_options} )
${sample_script} ${skillsim_options} -s discard \
	${gauss_params} > /dev/null 2>&1


