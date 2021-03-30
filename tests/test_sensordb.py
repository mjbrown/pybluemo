from pybluemo.sensordb import GetOrCreateDataSourceInput, CreateDataStreamsInput, CreateDataEventInput, SensorDbClient
from datetime import datetime, timedelta


def test_get_or_create(client):
    source_list = [GetOrCreateDataSourceInput("DE:AD:BE:EF:00:01", "Bluemo").json()]
    print(client.get_or_create_data_sources(source_list))


def test_create_data_streams(client):
    stream_input = [CreateDataStreamsInput("Test Data", "Chan", 0.1, "Eons", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
                    CreateDataStreamsInput('Moor Test', "Chan2", 0.1, "Eons", [0.7, 0.8, 0.9, 1.0, 1.1, 1.2])]
    result = client.save_data_stream(
        user_id="998ceb8a-e0f9-4bfb-ac37-537829909d02",
        device_id="dd442ac8-0386-406b-b783-53bd296a693e",
        start=datetime.utcnow().isoformat() + "Z",
        end=(datetime.utcnow() + timedelta(seconds=10 * 6)).isoformat() + "Z",
        stream_list=[i.json() for i in stream_input],
        linked_sources=[])
    print(result)
    pub_result = client.publish_streams(result)
    print(pub_result)


def test_create_data_event(client):
    event = CreateDataEventInput(
        user_id="998ceb8a-e0f9-4bfb-ac37-537829909d02",
        source_id="53fe69a6-9773-4094-ae5b-f7b2f291253d",
        label="Test Event #1")
    result = client.create_data_event(event)
    print(result)
    pub_result = client.publish_event(result['userId'], result['sourceId'], result['id'])
    print(pub_result)


if __name__ == "__main__":
    sensordb_client = SensorDbClient("../examples/sensordb_client_config.json")
    print(sensordb_client.list_cohorts())
    print(sensordb_client.list_sources())
    #test_get_or_create(sensordb_client)
    #test_create_data_streams(sensordb_client)
    test_create_data_event(sensordb_client)
