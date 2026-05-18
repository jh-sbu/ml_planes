use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

pub struct CsvLog {
    writer: BufWriter<File>,
}

impl CsvLog {
    pub fn new<P: AsRef<Path>>(path: P) -> std::io::Result<Self> {
        let f = File::create(path)?;
        Ok(Self {
            writer: BufWriter::new(f),
        })
    }

    pub fn write_comments(&mut self, comments: &[(&str, String)]) -> std::io::Result<()> {
        for (k, v) in comments {
            writeln!(self.writer, "# {k}={v}")?;
        }
        Ok(())
    }

    pub fn write_header(&mut self) -> std::io::Result<()> {
        writeln!(
            self.writer,
            "iter,steps,pct,elapsed_s,steps_per_sec,mean_return,ep_len,policy_loss,value_loss,entropy"
        )
    }

    pub fn write_row(
        &mut self,
        iter: usize,
        steps: usize,
        total_steps: usize,
        elapsed_s: f64,
        steps_per_sec: f64,
        mean_return: f32,
        ep_len: f32,
        policy_loss: f32,
        value_loss: f32,
        entropy: f32,
    ) -> std::io::Result<()> {
        let pct = 100.0 * steps as f64 / total_steps as f64;
        writeln!(
            self.writer,
            "{iter},{steps},{pct:.2},{elapsed_s:.2},{steps_per_sec:.1},{mean_return:.4},{ep_len:.1},{policy_loss:.6},{value_loss:.6},{entropy:.6}"
        )
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

    fn read_file(path: &str) -> String {
        let mut s = String::new();
        File::open(path).unwrap().read_to_string(&mut s).unwrap();
        s
    }

    #[test]
    fn write_comments_produces_hash_lines() {
        let path = "/tmp/csv_log_test_comments.csv";
        let mut log = CsvLog::new(path).unwrap();
        log.write_comments(&[
            ("task", "residual_orbit".to_string()),
            ("radial_reward_scale", "500".to_string()),
        ])
        .unwrap();
        log.flush().unwrap();

        let content = read_file(path);
        assert!(content.starts_with("# task=residual_orbit\n"));
        assert!(content.contains("# radial_reward_scale=500\n"));
    }

    #[test]
    fn write_header_produces_10_columns() {
        let path = "/tmp/csv_log_test_header.csv";
        let mut log = CsvLog::new(path).unwrap();
        log.write_header().unwrap();
        log.flush().unwrap();

        let content = read_file(path);
        let header = content.trim();
        assert_eq!(header.split(',').count(), 10, "header: {header}");
        assert!(header.starts_with("iter,steps,pct,"));
        assert!(header.ends_with(",entropy"));
    }

    #[test]
    fn write_row_produces_10_fields() {
        let path = "/tmp/csv_log_test_row.csv";
        let mut log = CsvLog::new(path).unwrap();
        log.write_row(
            1, 16384, 2_000_000, 3.40, 4823.1, 12.34, 3218.0, 0.042, 0.183, 1.204,
        )
        .unwrap();
        log.flush().unwrap();

        let content = read_file(path);
        let row = content.trim();
        assert_eq!(row.split(',').count(), 10, "row: {row}");
        assert!(row.starts_with("1,16384,"));
    }

    #[test]
    fn write_row_pct_computed_correctly() {
        let path = "/tmp/csv_log_test_pct.csv";
        let mut log = CsvLog::new(path).unwrap();
        // 500_000 / 2_000_000 = 25.00%
        log.write_row(
            10, 500_000, 2_000_000, 60.0, 8000.0, 5.0, 3000.0, 0.01, 0.05, 1.2,
        )
        .unwrap();
        log.flush().unwrap();

        let content = read_file(path);
        assert!(
            content.contains(",25.00,"),
            "expected 25.00 pct in: {content}"
        );
    }
}
