# Create BigQuery table

You can use this command to make a table with the specified schema
`bq mk --table swoopin-development:swoopin_iot.version_1 utils/schema.json`

You can delete previous schema with
`bq rm -f -t swoopin-development:swoopin_iot.version_1`

# Update device configuration
`
gcloud iot devices configs update --project=swoopin-development \
    --region=europe-west1 \
    --registry=swoopin-iot-registry \
    --device=esp32-lilygo-01 \
	--config-file=data/config.json
	`

# Cloud functions
The code in pubsub_to_bigquery.py send the data to a bigquery table, requirement.txt is related with it
