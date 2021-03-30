from datetime import datetime, timedelta
import boto3
import json
from aws_srp import AWSSRP
from python_graphql_client import GraphqlClient
import os


class GetOrCreateDataSourceInput(object):
    DEVICE_ID_COHORT = "1e340cc4-ee98-4617-a96f-1ea6ecc19e79"

    def __init__(self, object_key, sort_key, cohort_id=DEVICE_ID_COHORT):
        self.params = {
            "cohortId": cohort_id,
            "objectKey": object_key,
            "sortKey": sort_key
        }

    def json(self):
        return self.params


class CreateDataStreamsInput(object):
    def __init__(self, data_label, channel, frequency, unit, values):
        self.params = {
            "dataLabel": data_label,
            "channel": channel,
            "frequency": frequency,
            "unit": unit,
            "values": values
        }

    def json(self):
        return self.params


class CreateDataEventInput(object):
    def __init__(self, user_id, source_id, label, date_time=None, value=None, unit=None, channel=None):
        self.params = {
            "userId": user_id,
            "sourceId": source_id,
            "label": label
        }
        if date_time is not None:
            self.params['dateTime'] = date_time
        else:
            self.params['dateTime'] = datetime.utcnow().isoformat() + "Z"
        if value is not None:
            self.params['value'] = value
        if unit is not None:
            self.params['unit'] = unit
        if channel is not None:
            self.params['channel'] = channel

    def json(self):
        return self.params


class SensorDbClient(object):
    def __init__(self, cfg_json_filename, auth_token_filename="auth_token.json"):
        self.token = None
        with open(cfg_json_filename, "r") as cfg_fp:
            cfg_json = json.load(cfg_fp)
            username = cfg_json["Username"]
            password = cfg_json["Password"]
            pool_id = cfg_json["CognitoPoolId"]
            pool_region = cfg_json["CognitoPoolRegion"]
            client_id = cfg_json["CognitoClientId"]
            sensordb_api_url = cfg_json["SensorDbApiUrl"]
            if "CognitoClientSecret" in cfg_json:
                client_secret = cfg_json["CognitoClientSecret"]
            else:
                client_secret = None
        if os.path.isfile(auth_token_filename):
            with open(auth_token_filename, "r") as token_fp:
                result = json.load(token_fp)
                expires_in = timedelta(seconds=int(result['AuthenticationResult']['ExpiresIn']))
                header_date = datetime.strptime(result['ResponseMetadata']['HTTPHeaders']['date'], '%a, %d %b %Y %H:%M:%S GMT')
                print(header_date + expires_in, datetime.utcnow())
                if header_date + expires_in > datetime.utcnow():
                    self.token = result['AuthenticationResult']['AccessToken']
        if self.token is None:
            cognito_client = boto3.client('cognito-idp', pool_region)
            self.awssrp = AWSSRP(username, password, pool_id, client_id,
                                 client=cognito_client, client_secret=client_secret)
            result = self.awssrp.authenticate_user(client=cognito_client)
            with open(auth_token_filename, "w") as token_fp:
                json.dump(result, token_fp, indent=2)
            self.token = result['AuthenticationResult']['AccessToken']

        self.client = GraphqlClient(sensordb_api_url, headers={'authorization': self.token})

    def list_cohorts(self, query_filter=None, limit=None, next_token=None):
        query = """
query ListCohorts ($filter: ModelCohortFilterInput, $limit: Int, $nextToken: String) {
  listCohorts(filter: $filter, limit: $limit, nextToken: $nextToken) {
    items {
      id
      category
      collaborators
      createdAt
      label
      owner
      updatedAt
    }
    nextToken
  }
}"""
        variables = {}
        if query_filter is not None:
            variables['filter'] = query_filter
        if limit is not None:
            variables['limit'] = limit
        if next_token is not None:
            variables['nextToken'] = next_token
        result = self.client.execute(query=query, variables=variables)
        return result['data']

    def list_sources(self, query_filter=None, limit=None, next_token=None):
        query = """
query ListSources($filter: ModelDataSourceFilterInput, $limit: Int, $nextToken: String) {
  listDataSources(filter: $filter, limit: $limit, nextToken: $nextToken) {
    items {
      id
      objectKey
      sortKey
      cohortId
    }
    nextToken
  }
}"""
        variables = {}
        if query_filter is not None:
            variables['filter'] = query_filter
        if limit is not None:
            variables['limit'] = limit
        if next_token is not None:
            variables['nextToken'] = next_token
        result = self.client.execute(query=query, variables=variables)
        return result['data']

    def get_or_create_data_sources(self, mutation_input):
        mutation = """
mutation GetOrCreateDataSources($input: [GetOrCreateDataSourceInput]!) {
  getOrCreateDataSources(input: $input) {
    id
    metadata
    objectKey
    owner
    sortKey
    updatedAt
    cohortId
    collaborators
    createdAt
  }
}"""
        variables = {'input': mutation_input}
        result = self.client.execute(query=mutation, variables=variables)
        return result['data']

    def save_data_stream(self, user_id, device_id, start, end, stream_list, linked_sources=None):
        mutation = """
mutation SaveDataStreams($userId: ID!, $deviceId: ID!, $start: AWSDateTime!, $end: AWSDateTime!,
                         $dataStreams: [CreateDataStreamsInput]!, $linkedSources: [ID]) {
  saveDataStreams(userId: $userId, deviceId: $deviceId, start: $start, end: $end, 
                  dataStreams: $dataStreams, linkedSources: $linkedSources) {
    channel
    collaborators
    createdAt
    dataLabel
    end
    frequency
    id
    owner
    sourceId
    start
    unit
    updatedAt
    userId
    values
  }
}"""
        variables = {
            'userId': user_id,
            'deviceId': device_id,
            'start': start,
            'end': end,
            'dataStreams': stream_list
        }
        if linked_sources is not None:
            variables['linkedSources'] = linked_sources
        result = self.client.execute(query=mutation, variables=variables)
        return result['data']['saveDataStreams']

    def publish_streams(self, data_streams):
        mutation = """
mutation PublishStream($userId: ID!, $sourceId: ID!, $streamId: ID!) {
  publishStream(userId: $userId, sourceId: $sourceId, streamId: $streamId) {
    userId
    sourceId
    id
    start
    end
    dataLabel
    channel
    frequency
    unit
    values
  }
}"""
        results = []
        for stream in data_streams:
            variables = {
                "userId": stream["userId"],
                "sourceId": stream["sourceId"],
                "streamId": stream["id"]
            }
            result = self.client.execute(query=mutation, variables=variables)
            results.append(result)
        return results

    def create_data_event(self, create_data_event_input):
        mutation = """
mutation CreateDataEvent($input: CreateDataEventInput!, $condition: ModelDataEventConditionInput) {
  createDataEvent(input: $input, condition: $condition) {
    id
    userId
    sourceId
    dateTime
    label
    value
    unit
    channel
  }
}"""
        variables = {'input': create_data_event_input.json()}
        result = self.client.execute(query=mutation, variables=variables)
        return result['data']['createDataEvent']

    def publish_event(self, user_id, source_id, event_id):
        mutation = """
mutation PublishEvent($userId: ID!, $sourceId: ID!, $eventId: ID!) {
  publishEvent(userId: $userId, sourceId: $sourceId, eventId: $eventId) {
    id
    userId
    sourceId
    dateTime
    label
    value
    unit
    channel
  }
}"""
        variables = {'userId': user_id, 'sourceId': source_id, 'eventId': event_id}
        result = self.client.execute(query=mutation, variables=variables)
        return result
