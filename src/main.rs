use oddio::{Frames, Gain};
use oddio_engine::Engine;
use std::{
    thread,
    time::{Duration, Instant},
};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::f32::consts::PI;
use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;
use std::sync::{Arc, Mutex};
use wav::{BitDepth, Header};

const DURATION_SECS: u32 = 60;

fn main() {
    let host = cpal::default_host();
    let device = host
        .default_output_device()
        .expect("no output device available");
    let sample_rate = device.default_output_config().unwrap().sample_rate();
    let config = cpal::StreamConfig {
        channels: 2,
        sample_rate,
        buffer_size: cpal::BufferSize::Default,
    };

    let (mut scene_handle, scene) = oddio::spatial(sample_rate.0, 0.1);

    let stream = device
        .build_output_stream(
            &config.clone(),
            {
                let samples = Arc::new(Mutex::new(vec![]));
                let start = Instant::now();

                move |data: &mut [f32], _i: &cpal::OutputCallbackInfo| {
                    let frames = oddio::frame_stereo(data);
                    for s in &mut frames[..] {
                        *s = [0.0, 0.0];
                    }
                    oddio::run(&scene, sample_rate.0, frames);
                    samples.lock().unwrap().extend_from_slice(frames);

                    if false && start.elapsed().as_secs_f32() > 5.0 {
                        let _ = wav::write(
                            wav::Header {
                                sampling_rate: config.sample_rate.0,
                                bits_per_sample: 16,
                                audio_format: 1,
                                channel_count: 2,
                                bytes_per_sample: 2,
                                bytes_per_second: config.sample_rate.0 * 2 * 2,
                            },
                            &BitDepth::Sixteen(
                                samples
                                    .lock()
                                    .unwrap()
                                    .iter()
                                    .flat_map(|lr| {
                                        vec![
                                            (-lr[0] * (std::i16::MIN as i32 - 3) as f32)
                                                .min(std::i16::MAX as f32 - 1.0)
                                                .max(std::i16::MIN as f32 + 1.0)
                                                as i16,
                                            (-lr[1] * (std::i16::MIN as i32 - 3) as f32)
                                                .min(std::i16::MAX as f32 - 1.0)
                                                .max(std::i16::MIN as f32 + 1.0)
                                                as i16,
                                        ]
                                    })
                                    .collect(),
                            ),
                            &mut BufWriter::new(File::create("lol.wav").unwrap()),
                        );
                    }
                }
            },
            move |err| {
                eprintln!("{}", err);
            },
        )
        .unwrap();

    let (engine_signal, mut release_engine_handle, mut throttle_engine_handle) = {
        let release_rpms = [3900, 5100, 7440, 8520, 9180, 10020, 10500];

        let release_engine = Engine::new(
            3900.0,
            0.1,
            release_rpms.iter().map(|&rpm| {
                (rpm as f32, 0.003, {
                    let (header, data) = read_wav_f32(format!(
                        "canyoncar/Engine/loop_{}Rpm_release_engine.wav",
                        rpm
                    ));

                    Frames::from_slice(header.sampling_rate, &data)
                })
            }),
        );

        let throttle_rpms = [5280, 7380, 8520, 9180, 9600, 10500, 11520];

        let throttle_engine = Engine::new(
            3900.0,
            0.1,
            throttle_rpms.iter().map(|&rpm| {
                (rpm as f32, 0.003, {
                    let (header, data) = read_wav_f32(format!(
                        "canyoncar/Engine/loop_{}Rpm_throttle_engine.wav",
                        rpm
                    ));

                    Frames::from_slice(header.sampling_rate, &data)
                })
            }),
        );

        let (mut mixer_handle, mixer) = oddio::mixer();

        (
            mixer,
            mixer_handle.play(Gain::new(release_engine)),
            mixer_handle.play(Gain::new(throttle_engine)),
        )
    };

    let mut position = -8.0;
    let mut velocity = 0.0;

    let mut signal = scene_handle.play(
        engine_signal,
        //[-speed, 10.0, 0.0].into(),
        //[speed, 0.0, 0.0].into(),
        [position, 0.0, -8.0].into(),
        [velocity, 0.0, 0.0].into(),
        1000.0,
    );

    stream.play().unwrap();

    let start = Instant::now();
    let mut last_time = start;

    loop {
        thread::sleep(Duration::from_millis(10));

        let elapsed = start.elapsed();
        if elapsed >= Duration::from_secs(DURATION_SECS as u64) {
            break;
        }

        let dt = last_time.elapsed().as_secs_f32();
        last_time = Instant::now();

        velocity += 1.4 * dt;
        position += velocity * dt;

        signal
            .control::<oddio::Spatial<_>, _>()
            .set_motion([position, 0.0, -8.0].into(), [velocity, 0.0, 0.0].into());

        let phase = (elapsed.as_secs_f32() / 10.0 + 0.75).rem_euclid(1.0) * PI * 2.0;
        let rpm = 3800.0 + (phase.sin() * 0.5 + 0.5) * 10000.0;

        let shift = -0.5;
        let mix = (phase + (phase + shift).cos() * 0.8 + shift).cos() * 0.5 + 0.5;

        throttle_engine_handle
            .control::<Engine<_>, _>()
            .set_rpm(rpm);
        release_engine_handle.control::<Engine<_>, _>().set_rpm(rpm);
        throttle_engine_handle
            .control::<Gain<_>, _>()
            .set_gain(mix.sqrt());
        release_engine_handle
            .control::<Gain<_>, _>()
            .set_gain((1.0 - mix).sqrt());
    }
}

pub fn read_wav_f32<P: AsRef<Path>>(file: P) -> (Header, Vec<f32>) {
    let (header, data) = wav::read(&mut BufReader::new(
        File::open(file.as_ref())
            .expect(format!("Failed to open file '{}'", file.as_ref().display()).as_str()),
    ))
    .unwrap();

    let data: Vec<f32> = match data {
        BitDepth::Eight(bytes) => bytes.into_iter().map(|b| b as f32 / 255.0).collect(),
        BitDepth::Sixteen(ints) => ints
            .into_iter()
            .map(|b| b as f32 / -(std::i16::MIN as f32))
            .collect(),
        BitDepth::TwentyFour(ints) => ints
            .into_iter()
            .map(|b| b as f32 / -((1i32 << 24 - 1) as f32))
            .collect(),
        BitDepth::Empty => {
            vec![]
        }
    };

    (header, data)
}
