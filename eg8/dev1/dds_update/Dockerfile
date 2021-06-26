FROM python:3

RUN mkdir /dds
COPY ./cyclonedds.xml /dds
WORKDIR /usr/src/app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY ./dds_xml_updater.py .

CMD [ "python", "./dds_xml_updater.py" ]