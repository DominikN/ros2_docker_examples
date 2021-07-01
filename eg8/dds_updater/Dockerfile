FROM python:3

RUN mkdir /dds
COPY ./dds_updater/cyclonedds.xml /
WORKDIR /usr/src/app

COPY ./dds_updater/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY ./dds_updater//dds_xml_updater.py .

CMD [ "python", "./dds_xml_updater.py" ]