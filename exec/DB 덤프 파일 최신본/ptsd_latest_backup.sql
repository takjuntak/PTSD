--
-- PostgreSQL database dump
--

-- Dumped from database version 15.12 (Debian 15.12-1.pgdg120+1)
-- Dumped by pg_dump version 17.4

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET transaction_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

--
-- Name: notificationtype; Type: TYPE; Schema: public; Owner: ptsd
--

CREATE TYPE public.notificationtype AS ENUM (
    'start',
    'complete',
    'battery'
);


ALTER TYPE public.notificationtype OWNER TO ptsd;

--
-- Name: routine_type_enum; Type: TYPE; Schema: public; Owner: ptsd
--

CREATE TYPE public.routine_type_enum AS ENUM (
    'once',
    'daily'
);


ALTER TYPE public.routine_type_enum OWNER TO ptsd;

SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: devices; Type: TABLE; Schema: public; Owner: ptsd
--

CREATE TABLE public.devices (
    device_id integer NOT NULL,
    created_at timestamp without time zone NOT NULL,
    user_id integer NOT NULL,
    serial_number character varying(50) NOT NULL,
    name character varying(50) NOT NULL
);


ALTER TABLE public.devices OWNER TO ptsd;

--
-- Name: devices_device_id_seq; Type: SEQUENCE; Schema: public; Owner: ptsd
--

CREATE SEQUENCE public.devices_device_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.devices_device_id_seq OWNER TO ptsd;

--
-- Name: devices_device_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: ptsd
--

ALTER SEQUENCE public.devices_device_id_seq OWNED BY public.devices.device_id;


--
-- Name: notifications; Type: TABLE; Schema: public; Owner: ptsd
--

CREATE TABLE public.notifications (
    notification_id integer NOT NULL,
    user_id integer NOT NULL,
    "timestamp" timestamp without time zone NOT NULL,
    type public.notificationtype NOT NULL,
    title character varying(100),
    message text,
    is_read boolean NOT NULL
);


ALTER TABLE public.notifications OWNER TO ptsd;

--
-- Name: notifications_notification_id_seq; Type: SEQUENCE; Schema: public; Owner: ptsd
--

CREATE SEQUENCE public.notifications_notification_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.notifications_notification_id_seq OWNER TO ptsd;

--
-- Name: notifications_notification_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: ptsd
--

ALTER SEQUENCE public.notifications_notification_id_seq OWNED BY public.notifications.notification_id;


--
-- Name: routines; Type: TABLE; Schema: public; Owner: ptsd
--

CREATE TABLE public.routines (
    routine_id integer NOT NULL,
    user_id integer NOT NULL,
    start_time timestamp without time zone NOT NULL,
    routine_type public.routine_type_enum NOT NULL,
    is_work boolean NOT NULL,
    repeat_days integer[] NOT NULL
);


ALTER TABLE public.routines OWNER TO ptsd;

--
-- Name: routines_routine_id_seq; Type: SEQUENCE; Schema: public; Owner: ptsd
--

CREATE SEQUENCE public.routines_routine_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.routines_routine_id_seq OWNER TO ptsd;

--
-- Name: routines_routine_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: ptsd
--

ALTER SEQUENCE public.routines_routine_id_seq OWNED BY public.routines.routine_id;


--
-- Name: users; Type: TABLE; Schema: public; Owner: ptsd
--

CREATE TABLE public.users (
    user_id integer NOT NULL,
    email character varying NOT NULL,
    password character varying(128) NOT NULL,
    created_at timestamp without time zone,
    name character varying(50) NOT NULL
);


ALTER TABLE public.users OWNER TO ptsd;

--
-- Name: users_user_id_seq; Type: SEQUENCE; Schema: public; Owner: ptsd
--

CREATE SEQUENCE public.users_user_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.users_user_id_seq OWNER TO ptsd;

--
-- Name: users_user_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: ptsd
--

ALTER SEQUENCE public.users_user_id_seq OWNED BY public.users.user_id;


--
-- Name: devices device_id; Type: DEFAULT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.devices ALTER COLUMN device_id SET DEFAULT nextval('public.devices_device_id_seq'::regclass);


--
-- Name: notifications notification_id; Type: DEFAULT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.notifications ALTER COLUMN notification_id SET DEFAULT nextval('public.notifications_notification_id_seq'::regclass);


--
-- Name: routines routine_id; Type: DEFAULT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.routines ALTER COLUMN routine_id SET DEFAULT nextval('public.routines_routine_id_seq'::regclass);


--
-- Name: users user_id; Type: DEFAULT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.users ALTER COLUMN user_id SET DEFAULT nextval('public.users_user_id_seq'::regclass);


--
-- Data for Name: devices; Type: TABLE DATA; Schema: public; Owner: ptsd
--

COPY public.devices (device_id, created_at, user_id, serial_number, name) FROM stdin;
9	2025-05-15 08:43:32.401225	17	100000008e33ef3a	청소하는애
11	2025-05-16 04:49:44.141674	18	100000008e33ef3a	청소마루
17	2025-05-20 01:52:53.062886	7	100000008e33ef3a	짐콩이
\.


--
-- Data for Name: notifications; Type: TABLE DATA; Schema: public; Owner: ptsd
--

COPY public.notifications (notification_id, user_id, "timestamp", type, title, message, is_read) FROM stdin;
80	17	2025-05-21 08:07:02.071094	start	start	작업이 start되었습니다.	f
81	17	2025-05-21 08:07:02.082308	start	start	작업이 start되었습니다.	f
15	7	2025-05-18 05:55:06.281235	start	start	작업이 start되었습니다.	f
20	7	2025-05-18 06:07:32.996229	start	start	작업이 start되었습니다.	f
21	17	2025-05-19 13:48:41.572202	start	start	작업이 start되었습니다.	f
22	17	2025-05-19 13:52:54.652068	start	start	작업이 start되었습니다.	f
23	17	2025-05-19 13:58:08.691258	start	start	작업이 start되었습니다.	f
24	17	2025-05-19 13:59:09.54358	complete	complete	작업이 complete되었습니다.	f
25	17	2025-05-19 14:01:24.816044	start	start	작업이 start되었습니다.	f
27	17	2025-05-20 01:55:14.22051	complete	complete	작업이 complete되었습니다.	f
28	17	2025-05-20 01:55:14.224098	complete	complete	작업이 complete되었습니다.	f
29	17	2025-05-20 01:55:58.368302	start	start	작업이 start되었습니다.	f
30	17	2025-05-20 01:55:58.373176	start	start	작업이 start되었습니다.	f
31	17	2025-05-20 01:56:35.576849	complete	complete	작업이 complete되었습니다.	f
32	17	2025-05-20 01:56:35.582316	complete	complete	작업이 complete되었습니다.	f
33	17	2025-05-20 01:58:15.364844	complete	complete	작업이 complete되었습니다.	f
34	17	2025-05-20 01:58:15.366239	complete	complete	작업이 complete되었습니다.	f
37	17	2025-05-20 04:09:56.847124	start	start	작업이 start되었습니다.	f
38	17	2025-05-20 04:11:11.028618	start	start	작업이 start되었습니다.	f
39	17	2025-05-20 04:19:03.73223	start	start	작업이 start되었습니다.	f
40	17	2025-05-20 04:22:11.292134	start	start	작업이 start되었습니다.	f
41	17	2025-05-20 04:22:12.362424	complete	complete	작업이 complete되었습니다.	f
42	17	2025-05-20 04:40:15.219864	start	start	작업이 start되었습니다.	f
43	17	2025-05-20 05:48:42.984622	start	start	작업이 start되었습니다.	f
44	17	2025-05-20 05:48:54.000719	start	start	작업이 start되었습니다.	f
45	17	2025-05-20 05:48:55.357772	complete	complete	작업이 complete되었습니다.	f
46	17	2025-05-20 05:49:58.798399	complete	complete	작업이 complete되었습니다.	f
47	17	2025-05-20 08:59:31.317861	start	start	작업이 start되었습니다.	f
48	17	2025-05-20 09:01:17.068259	complete	complete	작업이 complete되었습니다.	f
49	17	2025-05-21 05:15:22.335306	start	start	작업이 start되었습니다.	f
50	17	2025-05-21 05:18:56.863606	start	start	작업이 start되었습니다.	f
51	17	2025-05-21 05:20:10.342624	complete	complete	작업이 complete되었습니다.	f
52	17	2025-05-21 05:22:14.531847	start	start	작업이 start되었습니다.	f
53	17	2025-05-21 05:23:48.753636	start	start	작업이 start되었습니다.	f
54	17	2025-05-21 05:43:14.824077	start	start	작업이 start되었습니다.	f
55	17	2025-05-21 05:43:14.830005	start	start	작업이 start되었습니다.	f
56	17	2025-05-21 05:46:05.377201	start	start	작업이 start되었습니다.	f
57	17	2025-05-21 05:46:05.338864	start	start	작업이 start되었습니다.	f
58	17	2025-05-21 05:47:29.896206	start	start	작업이 start되었습니다.	f
59	17	2025-05-21 05:47:29.858625	start	start	작업이 start되었습니다.	f
60	17	2025-05-21 05:48:44.654346	complete	complete	작업이 complete되었습니다.	f
61	17	2025-05-21 05:48:44.624961	complete	complete	작업이 complete되었습니다.	f
62	17	2025-05-21 05:49:52.900484	start	start	작업이 start되었습니다.	f
63	17	2025-05-21 05:49:52.856804	start	start	작업이 start되었습니다.	f
64	17	2025-05-21 05:51:32.35949	complete	complete	작업이 complete되었습니다.	f
65	17	2025-05-21 05:51:32.318073	complete	complete	작업이 complete되었습니다.	f
66	17	2025-05-21 05:54:08.268186	start	start	작업이 start되었습니다.	f
67	17	2025-05-21 05:54:08.2301	start	start	작업이 start되었습니다.	f
68	17	2025-05-21 05:55:29.614456	complete	complete	작업이 complete되었습니다.	f
69	17	2025-05-21 05:55:29.585509	complete	complete	작업이 complete되었습니다.	f
70	17	2025-05-21 05:55:39.798182	start	start	작업이 start되었습니다.	f
71	17	2025-05-21 05:55:39.759247	start	start	작업이 start되었습니다.	f
72	17	2025-05-21 05:56:51.594703	complete	complete	작업이 complete되었습니다.	f
73	17	2025-05-21 05:56:51.566569	complete	complete	작업이 complete되었습니다.	f
74	17	2025-05-21 05:58:03.461217	start	start	작업이 start되었습니다.	f
75	17	2025-05-21 05:58:03.421265	start	start	작업이 start되었습니다.	f
76	17	2025-05-21 05:59:54.954669	complete	complete	작업이 complete되었습니다.	f
77	17	2025-05-21 05:59:54.929013	complete	complete	작업이 complete되었습니다.	f
78	17	2025-05-21 06:02:38.046723	start	start	작업이 start되었습니다.	f
79	17	2025-05-21 06:02:38.001878	start	start	작업이 start되었습니다.	f
82	17	2025-05-21 08:07:02.502494	start	start	작업이 start되었습니다.	f
83	17	2025-05-21 08:08:39.79975	complete	complete	작업이 complete되었습니다.	f
84	17	2025-05-21 08:08:39.792171	complete	complete	작업이 complete되었습니다.	f
85	17	2025-05-21 08:08:39.785809	complete	complete	작업이 complete되었습니다.	f
86	17	2025-05-21 08:11:00.040752	start	start	작업이 start되었습니다.	f
87	17	2025-05-21 08:10:59.95696	start	start	작업이 start되었습니다.	f
88	17	2025-05-21 08:11:00.034494	start	start	작업이 start되었습니다.	f
89	17	2025-05-21 08:12:14.974845	complete	complete	작업이 complete되었습니다.	f
90	17	2025-05-21 08:12:14.896853	complete	complete	작업이 complete되었습니다.	f
91	17	2025-05-21 08:12:14.998401	complete	complete	작업이 complete되었습니다.	f
92	17	2025-05-21 08:14:19.793488	start	start	작업이 start되었습니다.	f
93	17	2025-05-21 08:14:19.715425	start	start	작업이 start되었습니다.	f
94	17	2025-05-21 08:14:19.839153	start	start	작업이 start되었습니다.	f
95	17	2025-05-21 08:16:22.947324	complete	complete	작업이 complete되었습니다.	f
96	17	2025-05-21 08:16:22.865323	complete	complete	작업이 complete되었습니다.	f
97	17	2025-05-21 08:16:22.950751	complete	complete	작업이 complete되었습니다.	f
98	17	2025-05-21 08:17:15.460629	start	start	작업이 start되었습니다.	f
99	17	2025-05-21 08:17:15.438902	start	start	작업이 start되었습니다.	f
100	17	2025-05-21 08:17:15.371721	start	start	작업이 start되었습니다.	f
101	17	2025-05-21 08:18:27.899823	complete	complete	작업이 complete되었습니다.	f
102	17	2025-05-21 08:18:27.815366	complete	complete	작업이 complete되었습니다.	f
103	17	2025-05-21 08:18:27.88793	complete	complete	작업이 complete되었습니다.	f
104	17	2025-05-21 08:18:50.243623	start	start	작업이 start되었습니다.	f
105	17	2025-05-21 08:18:50.226933	start	start	작업이 start되었습니다.	f
106	17	2025-05-21 08:18:50.161011	start	start	작업이 start되었습니다.	f
107	17	2025-05-21 08:20:09.984811	complete	complete	작업이 complete되었습니다.	f
108	17	2025-05-21 08:20:09.897248	complete	complete	작업이 complete되었습니다.	f
117	17	2025-05-21 08:34:00.024734	complete	complete	작업이 complete되었습니다.	f
118	17	2025-05-21 08:33:59.937148	complete	complete	작업이 complete되었습니다.	f
128	17	2025-05-21 08:39:23.255857	complete	complete	작업이 complete되었습니다.	f
129	17	2025-05-21 08:39:29.204536	start	start	작업이 start되었습니다.	f
138	17	2025-05-21 08:56:12.437704	start	start	작업이 start되었습니다.	f
109	17	2025-05-21 08:20:42.938252	start	start	작업이 start되었습니다.	f
110	17	2025-05-21 08:20:42.855238	start	start	작업이 start되었습니다.	f
111	17	2025-05-21 08:24:10.067397	start	start	작업이 start되었습니다.	f
119	17	2025-05-21 08:34:37.152268	complete	complete	작업이 complete되었습니다.	f
120	17	2025-05-21 08:34:37.058773	complete	complete	작업이 complete되었습니다.	f
121	17	2025-05-21 08:35:26.781944	start	start	작업이 start되었습니다.	f
123	17	2025-05-21 08:37:28.949327	complete	complete	작업이 complete되었습니다.	f
127	17	2025-05-21 08:39:23.345947	complete	complete	작업이 complete되었습니다.	f
130	17	2025-05-21 08:39:29.149379	start	start	작업이 start되었습니다.	f
142	17	2025-05-21 10:24:18.943539	complete	complete	작업이 complete되었습니다.	f
146	17	2025-05-21 10:47:14.12313	start	start	작업이 start되었습니다.	f
112	17	2025-05-21 08:24:09.98416	start	start	작업이 start되었습니다.	f
122	17	2025-05-21 08:35:26.700993	start	start	작업이 start되었습니다.	f
132	17	2025-05-21 08:40:52.188955	complete	complete	작업이 complete되었습니다.	f
113	17	2025-05-21 08:25:44.704509	complete	complete	작업이 complete되었습니다.	f
125	17	2025-05-21 08:38:23.809449	start	start	작업이 start되었습니다.	f
143	17	2025-05-21 10:38:39.142982	start	start	작업이 start되었습니다.	f
147	17	2025-05-21 10:50:42.26355	start	start	작업이 start되었습니다.	f
114	17	2025-05-21 08:25:44.621031	complete	complete	작업이 complete되었습니다.	f
124	17	2025-05-21 08:37:28.856946	complete	complete	작업이 complete되었습니다.	f
134	17	2025-05-21 08:53:24.712321	start	start	작업이 start되었습니다.	f
115	17	2025-05-21 08:30:11.860178	start	start	작업이 start되었습니다.	f
131	17	2025-05-21 08:40:52.287473	complete	complete	작업이 complete되었습니다.	f
137	17	2025-05-21 08:56:12.473766	start	start	작업이 start되었습니다.	f
141	17	2025-05-21 10:22:54.703053	start	start	작업이 start되었습니다.	f
145	17	2025-05-21 10:42:25.973406	start	start	작업이 start되었습니다.	f
149	17	2025-05-21 11:00:44.453178	start	start	작업이 start되었습니다.	f
151	17	2025-05-21 11:09:26.023295	start	start	작업이 start되었습니다.	f
153	17	2025-05-21 11:19:27.82322	start	start	작업이 start되었습니다.	f
155	17	2025-05-21 11:26:20.193298	start	start	작업이 start되었습니다.	f
116	17	2025-05-21 08:30:11.770767	start	start	작업이 start되었습니다.	f
126	17	2025-05-21 08:38:23.72006	start	start	작업이 start되었습니다.	f
133	17	2025-05-21 08:53:24.769298	start	start	작업이 start되었습니다.	f
135	17	2025-05-21 08:53:50.465226	start	start	작업이 start되었습니다.	f
136	17	2025-05-21 08:53:50.383199	start	start	작업이 start되었습니다.	f
139	17	2025-05-21 09:03:46.909734	start	start	작업이 start되었습니다.	f
140	17	2025-05-21 09:03:46.84748	start	start	작업이 start되었습니다.	f
144	17	2025-05-21 10:39:54.093445	complete	complete	작업이 complete되었습니다.	f
148	17	2025-05-21 10:52:07.12292	complete	complete	작업이 complete되었습니다.	f
150	17	2025-05-21 11:02:22.953657	complete	complete	작업이 complete되었습니다.	f
152	17	2025-05-21 11:11:03.683621	complete	complete	작업이 complete되었습니다.	f
154	17	2025-05-21 11:20:57.433483	complete	complete	작업이 complete되었습니다.	f
156	17	2025-05-21 11:28:33.663043	complete	complete	작업이 complete되었습니다.	f
157	17	2025-05-21 12:04:21.442605	start	start	작업이 start되었습니다.	f
158	17	2025-05-21 12:05:30.519778	complete	complete	작업이 complete되었습니다.	f
159	17	2025-05-21 12:05:38.172143	start	start	작업이 start되었습니다.	f
160	17	2025-05-21 12:06:57.66028	complete	complete	작업이 complete되었습니다.	f
161	17	2025-05-21 12:07:51.073789	start	start	작업이 start되었습니다.	f
162	17	2025-05-21 12:09:28.71272	complete	complete	작업이 complete되었습니다.	f
163	17	2025-05-21 12:12:58.830145	start	start	작업이 start되었습니다.	f
164	17	2025-05-21 12:18:44.203097	start	start	작업이 start되었습니다.	f
165	17	2025-05-21 12:42:29.099001	start	start	작업이 start되었습니다.	f
166	17	2025-05-21 12:44:07.261005	complete	complete	작업이 complete되었습니다.	f
167	17	2025-05-21 12:44:14.189962	start	start	작업이 start되었습니다.	f
168	17	2025-05-21 12:45:00.057797	start	start	작업이 start되었습니다.	f
169	17	2025-05-21 12:45:29.391428	complete	complete	작업이 complete되었습니다.	f
170	17	2025-05-21 12:47:00.043902	start	start	작업이 start되었습니다.	f
171	17	2025-05-21 13:40:00.061814	start	start	작업이 start되었습니다.	f
172	17	2025-05-21 15:27:11.577728	start	start	작업이 start되었습니다.	f
173	17	2025-05-21 15:54:29.432585	start	start	작업이 start되었습니다.	f
174	17	2025-05-21 15:55:49.062553	start	start	작업이 start되었습니다.	f
175	17	2025-05-21 15:56:16.812197	start	start	작업이 start되었습니다.	f
176	17	2025-05-21 15:58:28.342636	start	start	작업이 start되었습니다.	f
177	17	2025-05-21 15:59:39.082466	start	start	작업이 start되었습니다.	f
178	17	2025-05-21 16:00:50.942853	start	start	작업이 start되었습니다.	f
179	17	2025-05-21 16:02:57.114	start	start	작업이 start되었습니다.	f
180	17	2025-05-21 16:04:39.303625	start	start	작업이 start되었습니다.	f
181	17	2025-05-21 16:06:37.769876	start	start	작업이 start되었습니다.	f
182	17	2025-05-21 16:09:32.362807	start	start	작업이 start되었습니다.	f
183	17	2025-05-21 16:11:32.982929	start	start	작업이 start되었습니다.	f
184	17	2025-05-21 23:52:00.08701	start	start	작업이 start되었습니다.	f
185	17	2025-05-21 23:54:00.046536	start	start	작업이 start되었습니다.	f
\.


--
-- Data for Name: routines; Type: TABLE DATA; Schema: public; Owner: ptsd
--

COPY public.routines (routine_id, user_id, start_time, routine_type, is_work, repeat_days) FROM stdin;
111	7	2025-05-21 09:00:00	daily	f	{1,2}
112	7	2025-05-21 05:00:00	daily	f	{4,6}
131	17	2025-05-21 22:40:00	once	f	{}
90	18	2025-05-19 19:00:00	daily	f	{4}
91	18	2025-05-19 18:00:00	once	f	{}
101	18	2025-05-20 11:00:00	daily	f	{2,4}
124	18	2025-05-22 14:00:00	once	f	{}
\.


--
-- Data for Name: users; Type: TABLE DATA; Schema: public; Owner: ptsd
--

COPY public.users (user_id, email, password, created_at, name) FROM stdin;
17	ssafy1@naver.com	$2b$12$hae7ukm5VMhZFkm26pCNT.TI8A9FvdX8VgbjOMI3IPdi.QtmTJ3Tq	2025-05-15 08:42:45.379844	임정인
18	ssafy@naver.com	$2b$12$qAeItnVXN23SnTCTv4MaEuZSn3/Bwu5cGIH8M0riJhYU97AEUAadC	2025-05-15 08:52:31.958175	임정인
7	jjj@naver.com	$2b$12$yWKZ0r.F77GtJGzbTFjtdeq6OPyXV9X8D3RfllbV5TjZ/9/Uwz8/m	2025-05-13 04:37:43.962163	정준탁
29	jj@naver.com	$2b$12$QpsCqJZKjo8DnRspzO8Tj.wMccjy261PEV9HHRqnbNZEhJznr4cD6	2025-05-20 04:04:36.848859	원필
\.


--
-- Name: devices_device_id_seq; Type: SEQUENCE SET; Schema: public; Owner: ptsd
--

SELECT pg_catalog.setval('public.devices_device_id_seq', 20, true);


--
-- Name: notifications_notification_id_seq; Type: SEQUENCE SET; Schema: public; Owner: ptsd
--

SELECT pg_catalog.setval('public.notifications_notification_id_seq', 185, true);


--
-- Name: routines_routine_id_seq; Type: SEQUENCE SET; Schema: public; Owner: ptsd
--

SELECT pg_catalog.setval('public.routines_routine_id_seq', 138, true);


--
-- Name: users_user_id_seq; Type: SEQUENCE SET; Schema: public; Owner: ptsd
--

SELECT pg_catalog.setval('public.users_user_id_seq', 32, true);


--
-- Name: devices devices_pkey; Type: CONSTRAINT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.devices
    ADD CONSTRAINT devices_pkey PRIMARY KEY (device_id);


--
-- Name: notifications notifications_pkey; Type: CONSTRAINT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.notifications
    ADD CONSTRAINT notifications_pkey PRIMARY KEY (notification_id);


--
-- Name: routines routines_pkey; Type: CONSTRAINT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.routines
    ADD CONSTRAINT routines_pkey PRIMARY KEY (routine_id);


--
-- Name: users users_pkey; Type: CONSTRAINT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.users
    ADD CONSTRAINT users_pkey PRIMARY KEY (user_id);


--
-- Name: ix_routines_routine_id; Type: INDEX; Schema: public; Owner: ptsd
--

CREATE INDEX ix_routines_routine_id ON public.routines USING btree (routine_id);


--
-- Name: ix_users_email; Type: INDEX; Schema: public; Owner: ptsd
--

CREATE UNIQUE INDEX ix_users_email ON public.users USING btree (email);


--
-- Name: ix_users_user_id; Type: INDEX; Schema: public; Owner: ptsd
--

CREATE INDEX ix_users_user_id ON public.users USING btree (user_id);


--
-- Name: devices devices_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.devices
    ADD CONSTRAINT devices_user_id_fkey FOREIGN KEY (user_id) REFERENCES public.users(user_id) ON DELETE CASCADE;


--
-- Name: notifications notifications_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.notifications
    ADD CONSTRAINT notifications_user_id_fkey FOREIGN KEY (user_id) REFERENCES public.users(user_id) ON DELETE CASCADE;


--
-- Name: routines routines_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: ptsd
--

ALTER TABLE ONLY public.routines
    ADD CONSTRAINT routines_user_id_fkey FOREIGN KEY (user_id) REFERENCES public.users(user_id) ON DELETE CASCADE;


--
-- PostgreSQL database dump complete
--

